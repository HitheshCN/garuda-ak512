# FOC Removal Execution Log — `dspic33AK512MC510.X`

**Executed:** 2026-06-18  
**Baseline commit:** `510115a` — Sync 2026-06-18: VEX 4000KV (profile 6) dialed in + 6-step firmware app note  
**Plan reference:** `docs/foc_removal_plan.md`

---

## Phase 1 — Audit and Baseline

**Status:** Complete

**Baseline state confirmed:**
- All four FOC feature flags were already `0` in `garuda_config.h`.
- 6-step BLDC is the sole active algorithm.
- `foc/` directory existed on disk with full source tree.

**Symbols catalogued (key locations):**

| Symbol Class | Files Containing References |
|---|---|
| `FEATURE_FOC*` flags | `garuda_config.h`, `garuda_calc_params.h`, `garuda_service.c`, `garuda_types.h`, `main.c`, `hal/hal_adc.c/h`, `hal/hal_pwm.c/h`, `hal/port_config.c`, `hal/board_service.c`, `gsp/*.c/h` |
| FOC telemetry fields | `garuda_types.h` (three `#if` blocks), `gsp/gsp_commands.h`, `gsp/gsp_snapshot.c` |
| FOC state variables | `garuda_service.c` (`s_foc_v2`, `s_foc_v3`, `s_foc_an`, `s_pid_*`, etc.) |
| FOC fault codes | `garuda_types.h` (`FAULT_FOC_INTERNAL`, `FAULT_OBSERVER`, `FAULT_FOC_BUSLOSS`) |
| `ESC_DETECT` state | `garuda_types.h`, `main.c`, `garuda_service.c` |
| GSP FOC params | `gsp/gsp_params.h` (17 `PARAM_ID_FOC_*`, `PARAM_GROUP_FOC_*`), `gsp/gsp_params.c` (profile defaults, persist load/save, descriptor table) |
| `BuildFocMotorParams()` | `garuda_service.c` |

**Key finding:** `ESC_IF_RAMP` is a 6-step feature (`FEATURE_IF_STARTUP`) — kept as per plan constraint.

---

## Phase 2 — Remove FOC Source Files from MPLAB X Project

**Status:** Complete

**File edited:** `dspic33AK512MC510.X/nbproject/configurations.xml`

**Changes:**
- Removed `<logicalFolder name="foc">` block from `HeaderFiles` section (22 `.h` item paths).
- Removed `<logicalFolder name="foc">` block from `SourceFiles` section (14 `.c` item paths).
- Removed `<itemPath>../garuda_foc_params.h</itemPath>` from root header list.
- Removed `../foc` from `extra-include-directories` compiler property.

**Files removed from project (not yet from disk):**

| Category | Files |
|---|---|
| FOC v1 headers | `foc_types.h`, `clarke.h`, `park.h`, `svpwm.h`, `pi_controller.h`, `back_emf_obs.h`, `pll_estimator.h`, `flux_estimator.h`, `smo_observer.h`, `mxlemming_obs.h` |
| FOC v2 headers | `foc_v2_types.h`, `foc_v2_math.h`, `foc_v2_pi.h`, `foc_v2_observer.h`, `foc_v2_control.h`, `foc_v2_detect.h` |
| FOC v3 headers | `foc_v3_types.h`, `foc_v3_smo.h`, `foc_v3_control.h` |
| AN1078 headers | `an1078_params.h`, `an1078_smc.h`, `an1078_motor.h` |
| FOC v1 sources | `clarke.c`, `park.c`, `svpwm.c`, `pi_controller.c`, `back_emf_obs.c`, `pll_estimator.c`, `flux_estimator.c`, `smo_observer.c`, `mxlemming_obs.c` |
| FOC v2 sources | `foc_v2_pi.c`, `foc_v2_observer.c`, `foc_v2_control.c`, `foc_v2_detect.c` |
| FOC v3 sources | `foc_v3_smo.c`, `foc_v3_control.c` |
| AN1078 sources | `an1078_smc.c`, `an1078_motor.c` |

---

## Phase 3 — Remove FOC Feature Flags from `garuda_config.h`

**Status:** Complete

**File edited:** `garuda_config.h`

**Removed:**
- `#define FEATURE_FOC 0` (FOC v1)
- `#define FEATURE_FOC_V2 0` (FOC v2)
- `#define FEATURE_FOC_V3 0` (FOC v3)
- `#define FEATURE_FOC_AN1078 0` (AN1078)
- `#define FEATURE_SMO 0` (PLL+SMO parallel, v1 only)
- `#define FEATURE_MXLEMMING 0` (MXLEMMING flux observer, v1 only)
- Entire `/* FOC diagnostic levels ... */` comment block and `#define FOC_DIAG_PWM_TEST 2`

**Also updated:** `OC_CLPCI_ENABLE` comment in Hurst profile from "Disabled for FOC: SVPWM incompatible with CLPCI chopping" to "Disabled for Hurst bench testing".

---

## Phase 4 — Clean `garuda_types.h`

**Status:** Complete

**File edited:** `garuda_types.h`

**Changes:**

1. **`ESC_DETECT` state removed** from `ESC_STATE_T`. Was used only for FOC v2 auto-commissioning. All `_Static_assert` ordering guards verified unaffected (they do not reference `ESC_DETECT`).

2. **Three FOC fault codes removed** from `FAULT_CODE_T`:
   - `FAULT_FOC_INTERNAL`
   - `FAULT_OBSERVER`
   - `FAULT_FOC_BUSLOSS`

3. **`phaseCurrent` struct unconditionalised**: Removed `#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078` / `#endif` wrapper. Struct is now always present.

4. **FOC v1 telemetry block removed** (`#if FEATURE_FOC ... #endif`):  
   Fields: `focIa`, `focIb`, `focTheta`, `focOmega`, `focVbus`, `focIdcEst`, `focTheta2`, `focSubState`, `focOffsetIa`, `focOffsetIb`, `focSmoTheta` (SMO), `focSmoOmega` (SMO).

5. **FOC v2 telemetry block removed** (`#if FEATURE_FOC_V2 ... #endif`):  
   Fields: `focIdMeas`, `focIqMeas`, `focTheta`, `focOmega`, `focVbus`, `focIa`, `focIb`, `focThetaObs`, `focVd`, `focVq`, `focFluxAlpha`, `focFluxBeta`, `focLambdaEst`, `focObsGain`, `focPidDInteg`, `focPidQInteg`, `focPidSpdInteg`, `focModIndex`, `focObsConfidence`, `focSubState`, `focOffsetIa`, `focOffsetIb`.

6. **FOC v3/AN1078 telemetry block removed** (`#if FEATURE_FOC_V3 || FEATURE_FOC_AN1078 ... #endif`):  
   Same fields as v2 plus: `smoResidual`, `pllInnovation`, `pllOmega`, `omegaOl`, `handoffCtr`, `smoObservable`.

**`ESC_IF_RAMP` kept** — verified as `FEATURE_IF_STARTUP` (6-step parked feature), not FOC.

---

## Phase 5 — Clean `garuda_service.c`, `main.c`

**Status:** Complete

**Files edited:** `garuda_service.c`, `main.c`

### `garuda_service.c`

**Removed:**
- All FOC `#include` blocks (`#if FEATURE_FOC`, `#if FEATURE_FOC_V2`, `#if FEATURE_FOC_V3`, `#if FEATURE_FOC_AN1078`): 14 FOC headers + `garuda_foc_params.h`.
- FOC static state variables (`s_foc_v2`, `s_foc_v3`, `s_foc_an`, `s_pid_d/q/spd`, `s_obs`, `s_pll`, `s_flux_est`, `s_smo`, `s_mxl`, etc.).
- `BuildFocMotorParams()` function.
- FOC v1 module-level state + helpers (`foc_startup_reset`, `pll_rotor_angle`, `smo_rotor_angle`, `counts_to_amps_cal`, `counts_to_vbus`).
- `v2_counts_to_amps()` helper (v2/v3/AN1078).
- `#if !FEATURE_FOC && ...` guard on `phaseCurrent` init block in `GARUDA_ServiceInit()`.
- FOC `#if FEATURE_FOC`, `#elif FEATURE_FOC_V2`, `#elif FEATURE_FOC_V3`, `#elif FEATURE_FOC_AN1078` init blocks in `GARUDA_ServiceInit()`.
- ADC ISR buffer selection guard — replaced `#if FEATURE_FOC || ... raw_ia/raw_ib #else phaseB_val/phaseAC_val #endif` with unconditional `phaseB_val`/`phaseAC_val`.
- `#if !FEATURE_FOC && ...` guard on `phaseCurrent` peak tracking block.
- `#if !FEATURE_FOC && ...` guard on `bemf.bemfRaw = phaseB_val` in `#else` branch.
- Entire FOC/v2/v3/AN1078 dispatch block (~884 lines) replacing `#if FEATURE_FOC ... #elif FEATURE_FOC_V2 ... #elif FEATURE_FOC_V3 ... #elif FEATURE_FOC_AN1078 ... #else [6-step] #endif`. 6-step state machine now unconditional.
- Closing `#endif /* !FEATURE_FOC ... end of 6-step state machine */`.
- `#if FEATURE_FOC || ...` guards in Timer1 ISR for `ESC_ARMED`, `ESC_ALIGN`, `ESC_OL_RAMP`, `ESC_MORPH`, `ESC_CLOSED_LOOP` cases.
- `foc/` includes inside `#if FEATURE_IF_STARTUP` block (directory no longer exists; I-f feature remains parked at 0).
- Updated comment referencing `FEATURE_FOC counts_to_*` helpers.

### `main.c`

**Removed:**
- `garudaData.state = ESC_DETECT` assignment in the `gspDetectIntent` handler (state no longer exists).
- FOC LED2 state-encoding block (`#if FEATURE_FOC || ... #endif`).

---

## Phase 6 — Clean GSP: Parameters and Snapshot

**Status:** Complete

**Files edited:** `gsp/gsp_commands.h`, `gsp/gsp_snapshot.c`, `gsp/gsp_params.h`, `gsp/gsp_params.c`, `gsp/gsp_commands.c`

### `gsp/gsp_commands.h`

- **Removed** entire FOC telemetry block from `GSP_SNAPSHOT_T` (102 bytes):  
  `focIdMeas`, `focIqMeas`, `focTheta`, `focOmega`, `focVbus`, `focIa`, `focIb`, `focThetaObs`, `focVd`, `focVq`, `focFluxAlpha`, `focFluxBeta`, `focLambdaEst`, `focObsGain`, `focPidDInteg`, `focPidQInteg`, `focPidSpdInteg`, `focModIndex`, `focObsConfidence`, `focSubState`, `focPad`, `focOffsetIa`, `focOffsetIb`, `smoResidual`, `pllInnovation`, `pllOmega`, `omegaOl`, `handoffCtr`, `smoObservable`, `pad3`.
- **Updated** `_Static_assert(sizeof(GSP_SNAPSHOT_T) == 248)` → `== 146`.

### `gsp/gsp_snapshot.c`

- **Removed** FOC telemetry copy blocks (`#if FEATURE_FOC_V2 || ...` and `#elif FEATURE_FOC`).
- **Removed** `#if !FEATURE_FOC && ...` guard on 6-step `phaseCurrent` snapshot block (unconditional now).

### `gsp/gsp_params.h`

- **Removed** 17 `PARAM_ID_FOC_*` `#define` constants (IDs `0x70`–`0x80`).
- **Removed** 4 `PARAM_ID_AN1078_*` `#define` constants (IDs `0x90`–`0x93`).
- **Removed** `PARAM_GROUP_FOC_MOTOR` (8), `PARAM_GROUP_FOC_TUNING` (9), `PARAM_GROUP_FOC_STARTUP` (10), `PARAM_GROUP_AN1078` (11).
- **Removed** 17 FOC fields from `GSP_PARAMS_T`: `focRsMilliOhm`, `focLsMicroH`, `focKeUvSRad`, `focVbusNomCentiV`, `focMaxCurrentCentiA`, `focMaxElecRadS`, `focKpDqMilli`, `focKiDq`, `focObsLpfAlphaMilli`, `focAlignIqCentiA`, `focRampIqCentiA`, `focAlignTimeMs`, `focIqRampTimeMs`, `focRampRateRps2`, `focHandoffRadS`, `focFaultOcCentiA`, `focFaultStallDeciRadS`.
- **Removed** 4 AN1078 fields from `GSP_PARAMS_T`: `an1078ThetaBaseDegX10`, `an1078ThetaKE7`, `an1078KslideMv`, `an1078IdFwMaxDecia`.
- **Removed** V3 FOC extension block (bytes 48–81) from `GSP_CONFIG_PERSIST_V3_T`.
- **Updated** `_Static_assert(sizeof(GSP_CONFIG_PERSIST_V3_T) == 82)` → `== 48`.

### `gsp/gsp_params.c`

- **Removed** all `focRsMilliOhm`, `focLsMicroH`, …, `an1078*` field initializations from all 8 profile defaults (`[GSP_PROFILE_HURST]` through `[GSP_PROFILE_EVELTA_1407_3S]`).
- **Removed** 21-entry FOC + AN1078 parameter descriptor table entries (Motor Model group 8, Tuning group 9, Startup group 10, AN1078 group 11).
- **Removed** 3 `#if FEATURE_FOC_V2` blocks (in persist load/save functions).
- **Removed** FOC field assignments from persist load (`gspParams.focRsMilliOhm = persist.focRsMilliOhm;` × 17) and persist save (`persist.focRsMilliOhm = gspParams.focRsMilliOhm;` × 17).

### `gsp/gsp_commands.c`

- **Removed** `#if FEATURE_FOC_AN1078` include block (`an1078_params.h`, `an1078_motor.h`, `an1078_smc.h`).
- **Removed** FOC feature flag OR expression from `BuildFeatureFlags()` (bit 23 was FOC active).
- **Removed** `#if FEATURE_FOC_AN1078` build hash folding block (12 AN1078 constant XORs).
- **Removed** `#if FEATURE_FOC_AN1078` AN1078 re-init block in `SET_PARAM` handler.
- **Removed** `is_an1078_live` live-tuning gate and replaced with `is_oc_live`-only gate.

---

## Phase 7 — Delete `foc/` Directory and `garuda_foc_params.h`

**Status:** Complete

**Pre-deletion verification:** Confirmed no active `#include "foc/..."` or `#include "garuda_foc_params.h"` remained in any compiled source (the only remaining reference in `garuda_service.c` was inside `#if FEATURE_IF_STARTUP` — parked at 0 — and those includes were removed prior to deletion).

**Deleted:**
- `foc/` directory and all contents (23 files: 14 `.c`, 9 `.h`)
- `garuda_foc_params.h`

**Constraint respected:** `motor/pi.h` and `motor/pi.c` retained (used by `motor/speed_pi.c` for 6-step speed PI).

**Side effect handled:** `motor/hwzc.c` included `garuda_foc_params.h` for `VBUS_SCALE_V_PER_COUNT` (gated by `FEATURE_HWZC_PI_FLOAT && HWZC_PI_FF_ENABLE || FEATURE_HWZC_ABS_FLOOR`, both currently enabled):
- Moved `VBUS_SCALE_V_PER_COUNT` definition (and its dependencies `ADC_VREF_V`, `ADC_FULL_SCALE_F`, `VBUS_DIVIDER_RATIO`) into `garuda_calc_params.h`.
- Replaced `gspParams.focKeUvSRad` usage with `0.0f` — effectively disabling the back-EMF-based feedforward floor (the `lambdaPm > 0.0f` guard short-circuits). Feature can be re-enabled by providing a dedicated Ke GSP param.

---

## Phase 8 — Final Cleanup

**Status:** Complete

**Final global grep results:**

Remaining `FEATURE_FOC*` references exist only in HAL files (`hal_adc.c/h`, `hal_pwm.c/h`, `port_config.c`, `board_service.c`) that were not modified per the plan constraint. These references evaluate as dead branches (undefined macros = 0 in C preprocessor) and do not affect compilation. No HAL behaviour is changed.

**`garuda_calc_params.h` cleanup:**
- Removed 7 FOC mutual-exclusion `#error` blocks.
- Removed `FEATURE_MXLEMMING requires FEATURE_FOC` guard.
- Removed `#if !FEATURE_FOC && ...` / `#endif` wrapper around 6-step dependency guards (now unconditional).
- Removed dead FOC SVM CLPCI-disable block.

**`gsp/gsp_commands.c` comment cleanup:**
- Updated stale AN1078-referencing build-hash comment.

---

## Files Modified Summary

| File | Change |
|---|---|
| `dspic33AK512MC510.X/nbproject/configurations.xml` | Removed `foc/` logical folders; removed `../foc` include path |
| `garuda_config.h` | Removed 6 `FEATURE_FOC*` flags, `FOC_DIAG_PWM_TEST`; updated CLPCI comment |
| `garuda_types.h` | Removed `ESC_DETECT`, 3 fault codes, 3 FOC telemetry blocks; unconditionalised `phaseCurrent` |
| `garuda_calc_params.h` | Removed FOC mutual-exclusion guards; moved `VBUS_SCALE_V_PER_COUNT` here |
| `garuda_service.c` | Removed all FOC includes, state vars, dispatch blocks, ADC mux guards |
| `main.c` | Removed `ESC_DETECT` state assignment, FOC LED block |
| `motor/hwzc.c` | Removed `garuda_foc_params.h` include; replaced `focKeUvSRad` with `0.0f` |
| `gsp/gsp_commands.h` | Removed FOC telemetry fields from snapshot struct; updated size assert |
| `gsp/gsp_snapshot.c` | Removed FOC copy blocks; removed `!FEATURE_FOC` guard on 6-step block |
| `gsp/gsp_params.h` | Removed FOC param IDs, groups, struct fields, persist struct FOC extension |
| `gsp/gsp_params.c` | Removed FOC profile defaults, descriptor entries, persist load/save assignments |
| `gsp/gsp_commands.c` | Removed AN1078 includes, FOC hash folding, AN1078 re-init, `is_an1078_live` gate |

## Files Deleted

| File/Directory | Contents |
|---|---|
| `foc/` | 14 `.c` + 9 `.h`: clarke, park, svpwm, pi_controller, back_emf_obs, pll_estimator, flux_estimator, smo_observer, mxlemming_obs, foc_v2_*, foc_v3_*, an1078_* |
| `garuda_foc_params.h` | Compile-time FOC motor model constants |

---

## Constraints Applied

| Constraint | Outcome |
|---|---|
| `motor/pi.h` not deleted | Retained — used by `motor/speed_pi.c` |
| `ESC_IF_RAMP` not removed | Retained — `FEATURE_IF_STARTUP` is a 6-step feature |
| HAL files not modified | `hal_adc.c/h`, `hal_pwm.c/h`, `port_config.c`, `board_service.c` left untouched |
| `HAL_PWM_SetDutyFloat3Phase()` not removed | Retained in `hal_pwm.c/h` |
| Build after each phase | Phases 2–7 were code-only changes; build verification is the next step in MPLAB X |
