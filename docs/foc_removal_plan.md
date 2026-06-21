# FOC Removal Plan — `dspic33AK512MC510.X`

## Overview

**Objective:** Remove all four FOC variants (v1, v2, v3, AN1078) and their supporting infrastructure from the project without affecting the active 6-step BLDC algorithm, GSP, HAL, learn modules, or any other subsystem.

**Current state:** All four FOC feature flags are already `0` in `garuda_config.h`. The 6-step BLDC algorithm is the only compiled and executing code path. All FOC files exist on disk but are conditionally excluded from compilation via `#if` guards.

**Constraint at every phase:** Build and test after each phase before proceeding to the next.

---

## Algorithm Selection Reference

| Flag Set | Active Algorithm | Observer | Commutation | PWM Output |
|---|---|---|---|---|
| All FOC = 0 **(current)** | **6-step BLDC** | BEMF ZC (SW + HW) | `COMMUTATION_AdvanceStep()` | `HAL_PWM_SetDutyCycle()` (scalar) |
| `FEATURE_FOC_AN1078 = 1` | FOC AN1078 | AN1078 SMC + PLL | theta from SMO PLL | `HAL_PWM_SetDutyFloat3Phase()` |
| `FEATURE_FOC_V2 = 1` | FOC v2 | MXLEMMING + PLL | theta from MXLEMMING | `HAL_PWM_SetDutyFloat3Phase()` |
| `FEATURE_FOC_V3 = 1` | FOC v3 | Classical SMO + PLL | theta from SMO PLL | `HAL_PWM_SetDutyFloat3Phase()` |
| `FEATURE_FOC = 1` | FOC v1 (deprecated) | Back-EMF + PLL | theta from PLL | `HAL_PWM_SetDutyFloat3Phase()` |

The switching mechanism is **purely compile-time** (`#if / #elif` preprocessor blocks). There is no runtime algorithm selector.

---

## Phase Summary

| Phase | Action | Risk to 6-step | Key Files Touched |
|---|---|---|---|
| 1 | Audit and establish baseline | None | — |
| 2 | Remove FOC `.c` files from MPLAB X project | None (already guarded by `#if 0`) | `configurations.xml` |
| 3 | Remove FOC feature flags from `garuda_config.h` | Low — acts as a forced error detector | `garuda_config.h` |
| 4 | Remove FOC telemetry fields from `GARUDA_DATA_T` | Medium — touches shared struct | `garuda_types.h` |
| 5 | Remove FOC init and ISR dispatch branches | Medium — touches central service | `garuda_service.c` |
| 6 | Remove FOC GSP parameters and snapshot fields | Low | `gsp_params.c/.h`, `gsp_snapshot.c`, `gsp_commands.c` |
| 7 | Delete `foc/` directory and `garuda_foc_params.h` from disk | Low (already unlinked) | `foc/` directory, `garuda_foc_params.h` |
| 8 | Final cleanup, regression test, and commit | None | All previously touched files |

---

## Phase 1 — Audit and Baseline

**Goal:** Establish a verified, clean baseline before any deletions.

### Steps

1. Build the project in its current state and confirm zero errors and zero warnings.
2. Record the `.hex` file size and CRC as a regression baseline.
3. Run a full end-to-end motor test (arm → align → OL ramp → morph → CL) to confirm the 6-step path is fully functional.
4. Tag the current git commit as `pre-foc-removal-baseline`.
5. Grep the entire source tree for the symbols below and save the results to a tracking document. These locations must each be addressed in a later phase.

**Symbols to locate:**

- Feature flags: `FEATURE_FOC`, `FEATURE_FOC_V2`, `FEATURE_FOC_V3`, `FEATURE_FOC_AN1078`, `FEATURE_SMO`, `FEATURE_MXLEMMING`
- FOC file prefixes: `foc/`, `an1078_`, `foc_v2_`, `foc_v3_`, `clarke`, `park`, `svpwm`, `pi_controller`, `back_emf`, `pll_estimator`, `flux_estimator`, `smo_observer`, `mxlemming_obs`
- Telemetry fields: `focIa`, `focIb`, `focTheta`, `focOmega`, `focVbus`, `focIdMeas`, `focIqMeas`, `focThetaObs`, `focVd`, `focVq`, `focFluxAlpha`, `focFluxBeta`, `focLambdaEst`, `focObsGain`, `focPidDInteg`, `focPidQInteg`, `focPidSpdInteg`, `focModIndex`, `focSubState`, `focOffsetIa`, `focOffsetIb`, `smoResidual`, `pllInnovation`, `pllOmega`, `omegaOl`, `handoffCtr`, `smoObservable`
- States and faults: `ESC_DETECT`, `ESC_IF_RAMP`, `FAULT_FOC_INTERNAL`, `FAULT_OBSERVER`, `FAULT_FOC_BUSLOSS`
- Service-level identifiers: `BuildFocMotorParams`, `s_foc_an`, `s_foc_v2`, `s_foc_v3`
- GSP parameter IDs: `focRsMilliOhm`, `focLsMicroH`, `focKeVpeak`

### Exit Criteria

- Baseline build passes with zero errors and zero warnings.
- All reference locations are catalogued in the tracking document.

---

## Phase 2 — Remove FOC Source Files from MPLAB X Project

**Goal:** Unlink all FOC `.c` files from the project build so they no longer appear in `configurations.xml`.

### Files to Remove from the Project

> Do **not** delete these files from disk yet. Only remove them from the MPLAB X project source list.

| Subdirectory | Files |
|---|---|
| `foc/` | `clarke.c`, `park.c`, `svpwm.c`, `pi_controller.c`, `back_emf_obs.c`, `pll_estimator.c`, `flux_estimator.c`, `smo_observer.c`, `mxlemming_obs.c` |
| `foc/` | `foc_v2_pi.c`, `foc_v2_observer.c`, `foc_v2_control.c`, `foc_v2_detect.c` |
| `foc/` | `foc_v3_smo.c`, `foc_v3_control.c` |
| `foc/` | `an1078_smc.c`, `an1078_motor.c` |

### Steps

1. In `dspic33AK512MC510.X/nbproject/configurations.xml`, remove all `<cf name="foc/...">` source file entries for the files listed above.
2. Build and confirm the project still compiles cleanly. The existing `#if FEATURE_FOC == 0` guards mean none of these `.c` bodies are referenced.
3. Confirm no linker errors. FOC object files are not linked in already because the call sites are compiled out.

### Exit Criteria

- Project builds with zero errors.
- Only 6-step BLDC `.o` files appear in the linker map.

---

## Phase 3 — Remove FOC Feature Flags from `garuda_config.h`

**Goal:** Delete the six FOC feature-flag `#define` lines and all associated comments. Removing these flags causes the compiler to immediately error on any remaining FOC reference that was missed in later phases, making this phase act as a forced error detector.

### Lines to Remove

**`garuda_config.h:267–273`** — FOC feature block:

```c
/* FOC (Field-Oriented Control) — compile-time alternative to 6-step */
#define FEATURE_FOC              0  /* Phase I: OLD FOC v1 (reference, deprecated) */
#define FEATURE_FOC_V2           0  /* Phase I v2: closed-loop current control + MXLEMMING */
#define FEATURE_FOC_V3           0  /* Phase J: FOC v3 — SMO observer + PLL */
#define FEATURE_FOC_AN1078       0  /* 2026-05-25: switched to 6-step. */
#define FEATURE_SMO              0  /* 0=PLL only, 1=PLL+SMO parallel (v1 only) */
#define FEATURE_MXLEMMING        0  /* 0=PLL chain, 1=MXLEMMING flux observer (v1 only) */
```

**`garuda_config.h:320–327`** — FOC diagnostic block:

```c
/* FOC diagnostic levels (requires FEATURE_FOC=1): ... */
#define FOC_DIAG_PWM_TEST       2
```

### Steps

1. Delete the lines identified above.
2. Build. Any remaining `#if FEATURE_FOC` reference anywhere in the codebase will now produce a compile error. Use these errors to find and fix any missed cleanup from later phases.

### Exit Criteria

- Build passes.
- `FEATURE_FOC`, `FEATURE_FOC_V2`, `FEATURE_FOC_V3`, `FEATURE_FOC_AN1078`, `FEATURE_SMO`, `FEATURE_MXLEMMING`, and `FOC_DIAG_PWM_TEST` no longer exist as symbols.

---

## Phase 4 — Clean `garuda_types.h`: Remove FOC Fields from `GARUDA_DATA_T`

**Goal:** Remove the three FOC-conditional telemetry blocks from `GARUDA_DATA_T`, simplify the `phaseCurrent` struct guard, and remove FOC-specific states and fault codes.

### Removals in `garuda_types.h`

#### 1. FOC v1 telemetry block — `lines 756–772`

Remove the entire `#if FEATURE_FOC` block including:
`focIa`, `focIb`, `focTheta`, `focOmega`, `focVbus`, `focIdcEst`, `focTheta2`, `focSubState`, `focOffsetIa`, `focOffsetIb`, and the nested `#if FEATURE_SMO` block containing `focSmoTheta` and `focSmoOmega`.

#### 2. FOC v2 telemetry block — `lines 774–801`

Remove the entire `#if FEATURE_FOC_V2` block including:
`focIdMeas`, `focIqMeas`, `focTheta`, `focOmega`, `focVbus`, `focIa`, `focIb`, `focThetaObs`, `focVd`, `focVq`, `focFluxAlpha`, `focFluxBeta`, `focLambdaEst`, `focObsGain`, `focPidDInteg`, `focPidQInteg`, `focPidSpdInteg`, `focModIndex`, `focObsConfidence`, `focSubState`, `focOffsetIa`, `focOffsetIb`.

#### 3. FOC v3 / AN1078 telemetry block — `lines 803–837`

Remove the entire `#if FEATURE_FOC_V3 || FEATURE_FOC_AN1078` block including all fields listed above for v2 plus the additional SMO diagnostics: `smoResidual`, `pllInnovation`, `pllOmega`, `omegaOl`, `handoffCtr`, `smoObservable`.

#### 4. `phaseCurrent` struct guard — `line 682`

The `phaseCurrent` struct is currently wrapped in:

```c
#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078
    struct { ... } phaseCurrent;
#endif
```

Remove the `#if` / `#endif` wrapper entirely. The `phaseCurrent` struct is now unconditionally active for the 6-step path.

#### 5. `ESC_DETECT` state — `line 26`

`ESC_DETECT` is used only for `FEATURE_FOC_V2` auto-commissioning. Remove it from `ESC_STATE_T`. Remove any `_Static_assert` ordering guards that reference it, then re-verify the remaining asserts still hold.

#### 6. `ESC_IF_RAMP` state — `line 33`

> **Verify before removing.** Check `garuda_service.c` to confirm whether `ESC_IF_RAMP` is also used by `FEATURE_IF_STARTUP` (a 6-step feature). If it is used by 6-step, **keep it**. If it is FOC-only, remove it.

#### 7. FOC-specific fault codes — `lines 621–623`

Remove from `FAULT_CODE_T`:

```c
FAULT_FOC_INTERNAL,     /* FOC internal fault (estimator divergence, overcurrent) */
FAULT_OBSERVER,         /* Observer lost tracking (sustained reverse speed in CL) */
FAULT_FOC_BUSLOSS,      /* HW OC tripped: voltage applied but no current flows */
```

### Steps

1. Apply all removals above.
2. Build and confirm the compiler reports no reference to the removed fields.
3. Verify `sizeof(GARUDA_DATA_T)` has not introduced unexpected padding issues by checking with a `_Static_assert` or debugger watch.

### Exit Criteria

- `GARUDA_DATA_T` contains no FOC fields.
- `FAULT_CODE_T` contains no FOC fault codes.
- The `phaseCurrent` struct compiles unconditionally.
- The `ESC_DETECT` state is removed (subject to `ESC_IF_RAMP` verification).

---

## Phase 5 — Clean `garuda_service.c`: Remove FOC Init, ADC Mux Guards, and ISR Dispatch Branches

**Goal:** Remove all FOC-conditional code from the central service file without touching any 6-step logic.

### Removals in `garuda_service.c`

#### 1. FOC static state variables

Remove the following static variable declarations:

| Variable | FOC Variant |
|---|---|
| `s_foc_an` (`AN_Motor_t`) | AN1078 |
| `s_foc_v2` (`FOC_V2_State_t`) | v2 |
| `s_foc_v3` (`V3_State_t`) | v3 |
| `s_pid_d`, `s_pid_q`, `s_pid_spd` (`PI_t`) | v1 |
| `s_bemf_obs` (`BackEMFObs_t`) | v1 |
| `s_pll` (`PLL_t`) | v1 |
| `s_flux_est` (`FluxEst_t`) | v1 |
| `s_smo` (`SMO_t`) | v1 |
| `s_mxl` (`MxlObs_t`) | v1 |

#### 2. `BuildFocMotorParams()` function

Remove this entire function. It reads from `gspParams.focRs*` / compile-time FOC params, fills a `FOC_MotorParams_t`, and has no 6-step caller.

#### 3. `GARUDA_ServiceInit()` FOC initialisation blocks

Remove the following blocks in their entirety:

```c
#if FEATURE_FOC_V2
    foc_v2_init(&s_foc_v2, &mp);
#elif FEATURE_FOC_V3
    foc_v3_init(&s_foc_v3, &mp);
#elif FEATURE_FOC_AN1078
    AN_MotorInit(&s_foc_an);
#endif

#if FEATURE_FOC
    pi_init(&s_pid_d, ...);
    pi_init(&s_pid_q, ...);
    ...
#endif
```

#### 4. ADC ISR buffer selection guard

Simplify the ADC buffer selection from:

```c
#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
    uint16_t raw_ia = ADCBUF_IA;
    uint16_t raw_ib = ADCBUF_IB;
#else
    uint16_t phaseB_val  = ADCBUF_PHASE_B;
    uint16_t phaseAC_val = ADCBUF_PHASE_AC;
#endif
```

To the unconditional 6-step form (remove the `#if` / `#else` / `#endif` wrapper; keep only the `ADCBUF_PHASE_B` / `ADCBUF_PHASE_AC` lines).

#### 5. ADC ISR `ESC_CLOSED_LOOP` dispatch

Remove the FOC dispatch branches:

```c
#if FEATURE_FOC_AN1078
    AN_MotorFastTick(...);
    HAL_PWM_SetDutyFloat3Phase(...);
#elif FEATURE_FOC_V2
    foc_v2_fast_tick(...);
    HAL_PWM_SetDutyFloat3Phase(...);
#elif FEATURE_FOC_V3
    foc_v3_fast_tick(...);
    HAL_PWM_SetDutyFloat3Phase(...);
#else
    /* 6-step body */
#endif
```

Keep only the 6-step body. Remove the `#else` / `#endif` delimiters so the 6-step code is unconditional.

#### 6. `OC_CLPCI_ENABLE` comment update

Profile 0 (Hurst) in `garuda_config.h` carries the comment:
`"Disabled for FOC: SVPWM incompatible with CLPCI chopping"`

Update this to a neutral comment, for example:
`"Disabled for Hurst bench testing"`

#### 7. FOC `#include` directives

Remove all FOC header includes from `garuda_service.c`:

```c
#include "foc/foc_types.h"
#include "foc/foc_v2_types.h"
#include "foc/foc_v3_types.h"
#include "foc/foc_v2_control.h"
#include "foc/foc_v3_control.h"
#include "foc/an1078_motor.h"
#include "foc/pi_controller.h"
#include "foc/back_emf_obs.h"
#include "foc/pll_estimator.h"
#include "foc/flux_estimator.h"
#include "foc/smo_observer.h"
#include "foc/mxlemming_obs.h"
#include "garuda_foc_params.h"
```

### Steps

1. Apply all removals listed above.
2. Build and confirm zero errors.
3. Confirm no references to `s_foc_*`, `s_pid_d`, `BuildFocMotorParams`, or any FOC header remain.

### Exit Criteria

- `garuda_service.c` contains no FOC-conditional branches.
- The ADC ISR reads only BEMF phase voltage channels.
- The initialisation path is 6-step only.

---

## Phase 6 — Clean GSP: Remove FOC Runtime Parameters and Snapshot Fields

**Goal:** Remove the 17 FOC runtime GSP parameters (IDs `0x70`–`0x86`) from the parameter table and the snapshot builder.

### Removals

#### `gsp/gsp_params.c`

Remove all `GSP_PARAM_FOC_*` entries from the `profileDefaults[]` array and the parameter descriptor table. These are identified by parameter IDs `0x70` through `0x86`, covering:
`focRsMilliOhm`, `focLsMicroH`, `focKeVpeak`, and all other FOC motor model and PI gain parameters.

#### `gsp/gsp_params.h`

Remove all `GSP_PARAM_FOC_*` enum values and `#define` ID constants.

#### `gsp/gsp_snapshot.c`

Remove any `#if FEATURE_FOC*` blocks that pack FOC telemetry fields into snapshot packets, including:
`focIdMeas`, `focIqMeas`, `focTheta`, `focOmega`, `focVd`, `focVq`, and all other FOC-specific telemetry fields.

Update any snapshot packet size documentation or `_Static_assert` size checks to reflect the reduced field count.

#### `gsp/gsp_commands.c`

Remove any `case GSP_PARAM_FOC_*` dispatch entries in the command handler.

### Steps

1. Apply all removals.
2. Build and verify no GSP handler references any removed parameter ID.
3. Confirm the GSP snapshot size assertion (if present) still matches the reduced field count.

### Exit Criteria

- The GSP parameter table contains no FOC entries.
- The snapshot builder produces no FOC telemetry fields.
- The GSP protocol surface exposed to a connected tuning tool is clean.

---

## Phase 7 — Delete `foc/` Directory and `garuda_foc_params.h` from Disk

**Goal:** Physically remove all FOC source and header files from the repository.

### Files to Delete

```
foc/
├── foc_types.h
├── foc_v2_math.h
├── clarke.c / clarke.h
├── park.c / park.h
├── svpwm.c / svpwm.h
├── pi_controller.c / pi_controller.h
├── back_emf_obs.c / back_emf_obs.h
├── pll_estimator.c / pll_estimator.h
├── flux_estimator.c / flux_estimator.h
├── smo_observer.c / smo_observer.h
├── mxlemming_obs.c / mxlemming_obs.h
├── foc_v2_types.h
├── foc_v2_pi.c / foc_v2_pi.h
├── foc_v2_observer.c / foc_v2_observer.h
├── foc_v2_control.c / foc_v2_control.h
├── foc_v2_detect.c / foc_v2_detect.h
├── foc_v3_types.h
├── foc_v3_smo.c / foc_v3_smo.h
├── foc_v3_control.c / foc_v3_control.h
├── an1078_params.h
├── an1078_smc.c / an1078_smc.h
└── an1078_motor.c / an1078_motor.h

garuda_foc_params.h
```

> **Important:** `motor/pi.h` (`MC_PI_T`) is used by `an1078_motor.c` but is **also used by `motor/speed_pi.c`** (the 6-step speed PI path). Do **not** delete `motor/pi.h`.

### Steps

1. Confirm `configurations.xml` no longer references any of these files (completed in Phase 2).
2. Grep the entire project for any remaining `#include "foc/`, `#include "garuda_foc_params`, or `#include "an1078_` directives to verify no stragglers exist.
3. Delete the `foc/` directory and `garuda_foc_params.h`.
4. Build and confirm zero errors.

### Exit Criteria

- The `foc/` directory no longer exists on disk.
- `garuda_foc_params.h` no longer exists on disk.
- Build succeeds.
- Binary size is measurably smaller than the Phase 1 baseline.

---

## Phase 8 — Final Cleanup, Regression Test, and Commit

**Goal:** Eliminate all remaining FOC artifacts, verify full system integrity, and produce a clean commit.

### Remaining Cleanup Tasks

1. **`OC_CLPCI_ENABLE` comment** — Verify the Hurst profile comment updated in Phase 5 no longer contains any FOC reference.

2. **`garuda_types.h` comment sweep** — Confirm no comment-only references to `FAULT_FOC_INTERNAL`, `FAULT_OBSERVER`, `FAULT_FOC_BUSLOSS`, or `ESC_DETECT` remain in any file.

3. **`FOC_DIAG_PWM_TEST` residual** — Confirm the macro is fully removed and no `switch (FOC_DIAG_PWM_TEST)` dead-code block survives in any file.

4. **`docs/` folder** (optional, team discretion) — Remove or archive FOC-specific migration notes, design documents, or session notes that are no longer relevant.

5. **Final global grep** — Search the entire source tree for `FOC`, `foc_`, `an1078`, `clarke`, `svpwm`, `MXLEMMING`, `FEATURE_SMO`. Any hit that is not inside a comment describing the removal should be investigated.

### Regression Test

1. Repeat the full end-to-end motor test from Phase 1 on all motor profiles used in CI and bench testing.
2. Confirm identical startup behaviour (arm → align → OL ramp → morph → CL), idle speed, and maximum eRPM versus the Phase 1 baseline.
3. Verify all fault paths (overcurrent, desync, voltage fault) still trigger correctly.

### Binary Validation

Compare the output `.hex` against the Phase 1 baseline.

- The 6-step code section should be **bit-for-bit identical**. The compiler must produce the same object for the 6-step path.
- Any difference must be explained — for example, removal of previously unused-but-linked symbols causing linker address shifts, or removal of unreferenced string literals.
- A smaller `.hex` is expected and confirms FOC code was successfully stripped.

### Git Commit

```
git add -A
git commit -m "Remove FOC algorithm (all variants v1/v2/v3/AN1078) — 6-step BLDC is the sole algorithm"
```

### Exit Criteria

- Zero compiler warnings or errors.
- Regression test passes on all motor profiles.
- Binary diff confirms 6-step section is identical to the Phase 1 baseline.
- No FOC symbols remain in the compiled output (verify via `nm` or MPLAB X symbol map).

---

## Critical Constraints

| Constraint | Detail |
|---|---|
| Do not delete `motor/pi.h` | Used by `motor/speed_pi.c` (6-step speed PI path) as well as the now-removed `an1078_motor.c` |
| Verify `ESC_IF_RAMP` before removing | `FEATURE_IF_STARTUP` (a 6-step option) may use this state — check `garuda_service.c` before deleting |
| Build after every phase | The `#if 0` guards mean Phase 2 is safe, but Phases 3–7 each touch live code paths |
| Do not modify HAL | `HAL_PWM_SetDutyFloat3Phase()` can remain in `hal_pwm.c` / `hal_pwm.h` — it is unused after FOC removal but causes no harm and avoids risk |
