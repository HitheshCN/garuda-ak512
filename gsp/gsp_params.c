/**
 * @file gsp_params.c
 *
 * @brief Runtime parameter system implementation (Phase 1.5).
 *
 * Table-driven validation, cross-parameter checks, derived value
 * precomputation, profile defaults, and EEPROM V2 persistence.
 *
 * Component: GSP
 */

#include "garuda_config.h"

#if FEATURE_GSP

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "gsp_params.h"
#include "garuda_calc_params.h"

/* â”€â”€ Global instances â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

GSP_PARAMS_T  gspParams;
GSP_DERIVED_T gspDerived;

static uint8_t activeProfile;

/* â”€â”€ Profile defaults (hardcoded, not macro-derived) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 * garuda_config.h only exposes one profile at compile time via #if.
 * These const arrays duplicate the literal values from each profile
 * block â€” intentional: stable per-motor constants that rarely change. */

/* Shared tuning defaults (same for all profiles) */
#define TUNING_DEFAULTS \
    .dutySlewUpPctPerMs   = 2,  \
    .dutySlewDownPctPerMs = 5,  \
    .postSyncSettleMs     = 1000, \
    .postSyncSlewDivisor  = 4,  \
    .zcBlankingPercent    = 3,  \
    .zcAdcDeadband        = 4,  \
    .zcSyncThreshold      = 6,  \
    .zcFilterThreshold    = 2,  \
    .vbusOvAdc            = 3600, \
    .vbusUvAdc            = 500,  \
    .desyncCoastMs        = 200,  \
    .desyncMaxRestarts    = 3,  \
    .morphLockZcCount     = 4,  \
    .morphLockTolPct      = 25,  \
    .ifCurrentCa          = 600, \
    .ifRampErpmPerS       = 12000

static const GSP_PARAMS_T profileDefaults[9] = {
    [GSP_PROFILE_HURST] = {
        .rampTargetErpm     = 2000,
        .rampAccelErpmPerS  = 1000,
        .rampDutyPct        = 40,
        .clIdleDutyPct      = 0,
        .timingAdvMaxDeg    = 15,
        .hwzcCrossoverErpm  = 5000,
        .ocSwLimitMa        = 1500,
        .ocFaultMa          = 3000,
        .motorPolePairs     = 5,
        .alignDutyPct       = 20,
        .initialErpm        = 300,
        .maxClosedLoopErpm  = 20000,
        .sineAlignModPct    = 15,
        .sineRampModPct     = 35,
        .zcDemagDutyThresh  = 70,
        .zcDemagBlankExtraPct = 12,
        .ocLimitMa          = 1800,
        .ocStartupMa        = 18000,
        .rampCurrentGateMa  = 0,
        TUNING_DEFAULTS,
    },
    [GSP_PROFILE_A2212] = {
        .rampTargetErpm     = 3000,
        .rampAccelErpmPerS  = 3000,    /* Faster ramp for bench (200â†’3000 eRPM in ~1s) */
        .rampDutyPct        = 15,
        .clIdleDutyPct      = 8,     /* 2026-06-16 12->8: CL idle duty is the startup-inrush driver
                                      * (duty vs near-zero low-speed BEMF on 0.065R A2212). 12%=~14A,
                                      * 8% ~9A. Idle speed drops ~13k->~9k. Raise if ZC roughens. */
        .timingAdvMaxDeg    = 23,    /* 2026-06-16 back to 23 (26 over-advanced, 20 no better) â€” sweet
                                      * spot near the prior 22. Ramps 0deg@3k -> 23deg@maxCLerpm.
                                      * Live-tunable (PARAM_ID_TIMING_ADV_MAX_DEG) â€” A/B 22 vs 23 live. */
        .hwzcCrossoverErpm  = 1500,
        .ocSwLimitMa        = 15000, /* Bumped from 8000 â€” avg bench current low */
        .ocFaultMa          = 22000, /* Bumped from 18000 â€” leave headroom above CMP3 */
        .motorPolePairs     = 7,
        .alignDutyPct       = 8,
        .initialErpm        = 200,
        .maxClosedLoopErpm  = 120000,
        .sineAlignModPct    = 4,
        .sineRampModPct     = 12,
        .zcDemagDutyThresh  = 40,
        .zcDemagBlankExtraPct = 18,
        .ocLimitMa          = 18000, /* Bumped from 12000 â€” CMP3 tolerates commutation spikes */
        .ocStartupMa        = 25000, /* Bumped slightly */
        .rampCurrentGateMa  = 5000,
        TUNING_DEFAULTS,
    },
    [GSP_PROFILE_5010] = {
        /* === 2810 1350KV (7-8" FPV/cine drone motor, 24V bench) ===
         * Motor data from PATA6847/CK board project (garuda_6step_ck.X,
         * MOTOR_PROFILE=2). 12N14P, 7PP, 5-6S LiPo (18.5-25.2V), Rs~50mÎ©,
         * Ls~25ÂµH. At 24V: no-load eRPM ceiling = 1350*24*7 = 226.8k.
         * Target: 200k eRPM bench no-prop.
         *
         * Slot name is still GSP_PROFILE_5010 for EEPROM/profile-id
         * compatibility; actual motor is 2810 here (AKESC's original
         * 5010 data is in PATA comments/backup). */
        .rampTargetErpm     = 3000,    /* Reliable OL ramp endpoint on 2810 at 24V.
                                        * The iron is the ceiling, not the accel: low-L
                                        * (25ÂµH)/low-Rs(50mÎ©) 2810 can't be open-loop-
                                        * dragged past ~3-4k synced (5k/6k retries with
                                        * gentler accel still slipped -> 184k phantom +
                                        * stall, 2026-06-07). The ~19-22A CL-entry pulse
                                        * (3k rotor vs ~10.4k CL idle) is the accepted
                                        * tradeoff; bounded by OC_SW, brief, non-damaging. */
        .rampAccelErpmPerS  = 3000,    /* ~1s ramp 200->3000 eRPM on 2810 at 24V. */
        .rampDutyPct        = 8,       /* At 24V, 8% * 24V / 0.050Î© = 38A stall.
                                        * Motor should be moving early; 8% is ramp cap */
        .clIdleDutyPct      = 4,       /* With FEATURE_HWZC_LOWSPD_OFFCTR=1, rising ZC no
                                        * longer needs the PWM-ON window, so idle holds
                                        * below the old 6% floor: this clamps to MIN_DUTY
                                        * -> ~10.4k idle (was ~14.3k at 6%), shrinking the
                                        * 3k-handoff gap and trimming the pulse ~22->~19.6A.
                                        * Lowering further is a dead end (idle is floored
                                        * by MIN_DUTY in the trap waveform). Was 6. */
        .timingAdvMaxDeg    = 20,      /* AK512 bench 2026-06-13: 25Â° OVER-advances ->
                                        * falling-ZC sectors lost >210k -> desync/UV at
                                        * ~214k on accel. 20Â° reaches full 260k cap
                                        * (AK128 parity), holds clean. 10Â° also safe but
                                        * less mid-band advance. (Remaining hard-decel-chop
                                        * desync ~220k is the duty-down-slew issue, not advance.) */
        .hwzcCrossoverErpm  = 1500,    /* Enable HWZC immediately after morph */
        .ocSwLimitMa        = 18000,   /* Soft limit. Board shunt saturates ~22A */
        .ocFaultMa          = 21000,   /* SW hard fault just below sensor saturation */
        .motorPolePairs     = 7,
        .alignDutyPct       = 3,       /* 24V * 3% / 0.050Î© = 14.4A stall.
                                        * Half of A2212's 8% at 12V for similar current */
        .initialErpm        = 150,     /* Slow start â€” 2810 needs gentle first steps */
        .maxClosedLoopErpm  = 260000,  /* NOTE: this is the timing-ADVANCE anchor (eRPM where advance
                                        * reaches timingAdvMaxDeg), NOT a speed limiter â€” lowering it
                                        * steepens the advance ramp and over-advances the top band.
                                        * Keep at 260k for the clean schedule. Raised 220â†’260k (2026-05-26). Old 220k cap
                                        * was exactly the BEMF ceiling for 92% duty
                                        * at 24V. Motor was getting stuck at the
                                        * cap, not BEMF â€” current spikes at 93%+
                                        * caused by trying to push duty against a
                                        * speed lock. 260k = ~118% of theoretical
                                        * BEMF ceiling. If motor stays at <230k,
                                        * BEMF is the true limit. */
        .sineAlignModPct    = 3,       /* Conservative align â€” low Rs means current */
        .sineRampModPct     = 5,       /* Conservative for low-Rs 2810 (0.05Î©). 5% mod ~5.6A
                                        * peak in OL_RAMP. Retry at 8% (2026-06-07) raised
                                        * peak to ~12.5A but rotor still slipped past ~4k â€”
                                        * amplitude is not the limit, the iron is. Was 5. */
        .zcDemagDutyThresh  = 40,      /* Same as A2212 â€” low L â†’ more demag */
        .zcDemagBlankExtraPct = 20,    /* Aggressive demag blanking (low L = long tail) */
        .ocLimitMa          = 20000,   /* 2026-06-17 reverted 600->20000 to the committed 2810
                                        * baseline (CMP3 chop parked just below sensor saturation;
                                        * the 600 value was the bring-up chop experiment). */
        .ocStartupMa        = 22000,   /* Startup relaxed near sensor saturation */
        .rampCurrentGateMa  = 10000,   /* Gate ramp accel if bus >10A during OL */
        TUNING_DEFAULTS,
                                       
        /* AN1078 SMC tuning â€” values validated on 2810 @ 45 kHz.
         * BASE Ã— 10 = 200 â†’ 20Â° at zero speed.
         * K Ã— 1e7 = 800 â†’ 8.0e-5 rad/(rad/s elec) â€” re-tuned for 45 kHz
         *   2026-04-26.  At 60 kHz K=1000 was the value; at 45 kHz the
         *   LPF lag profile shifts and 800 keeps Vd within Â±2V across
         *   the 50k â†’ 200k+ range.  K=500 left Vd at -10V (heroic d-PI
         *   compensation) and K=1000 at +6V (over-corrected).
         * Kslide Ã— 1000 = 2500 â†’ 2.5 V.
         * |Id_FW_max| Ã— 10 = 120 â†’ -12 A (12 A peak field-weakening). */
    },
    [GSP_PROFILE_5055] = {
        .rampTargetErpm     = 2000,
        .rampAccelErpmPerS  = 150,
        .rampDutyPct        = 8,
        .clIdleDutyPct      = 8,
        .timingAdvMaxDeg    = 15,
        .hwzcCrossoverErpm  = 1500,
        .ocSwLimitMa        = 10000,
        .ocFaultMa          = 20000,
        .motorPolePairs     = 7,
        .alignDutyPct       = 4,
        .initialErpm        = 100,
        .maxClosedLoopErpm  = 80000,
        .sineAlignModPct    = 4,
        .sineRampModPct     = 8,
        .zcDemagDutyThresh  = 45,
        .zcDemagBlankExtraPct = 16,
        .ocLimitMa          = 15000,
        .ocStartupMa        = 22000,
        .rampCurrentGateMa  = 6000,
        TUNING_DEFAULTS,
    },
    [GSP_PROFILE_COBRA] = {
        /* === Cobra CM-2814/36 470KV (12N14P, 7PP, 4-6S, 117g, 36T delta) ===
         * Rs(phase-phase)=0.188Î©, KV=470, max cont 17A. Numbers for 24V/6S bench.
         * OPPOSITE regime to the 2810: ~4Ã— the resistance + heavy 117g rotor +
         * low KV. So the sine startup needs MUCH more amplitude (the real torque
         * knob is sineAlign/RampModPct, NOT the trap duty %) and a slower ramp.
         * Low KV = strong BEMF = ZC detection is easy. These are PHYSICS-BASED
         * STARTING values â€” Ls is UNMEASURED; iterate from the GSP fault code
         * (ALIGN/OL_RAMP stall â†’ raise sineRampModPct; desync â†’ lower rampAccel;
         * OC â†’ lower amplitude). Built from the 2810 entry; only regime fields
         * changed. With FEATURE_GSP=1 this table is the runtime source. */
        .rampTargetErpm     = 3000,    /* keep â€” strong BEMF at 470KV, easy handoff */
        .rampAccelErpmPerS  = 1000,    /* slow: heavy rotor needs dwell per step (~3s ramp) */
        .rampDutyPct        = 12,      /* MORPH/trap duty cap â€” higher R needs more */
        .clIdleDutyPct      = 8,       /* 24V*8%/0.188Î© idle authority for heavy rotor */
        .timingAdvMaxDeg    = 18,      /* lower top speed than 2810 â†’ less advance */
        .hwzcCrossoverErpm  = 1500,
        .ocSwLimitMa        = 16000,   /* soft limit â‰ˆ rated 17A continuous */
        .ocFaultMa          = 21000,
        .motorPolePairs     = 7,
        .alignDutyPct       = 8,       /* trap align cap (sine path uses sineAlignModPct) */
        .initialErpm        = 100,     /* gentle first step for high inertia */
        .maxClosedLoopErpm  = 83000,   /* 470 * 24V * 7pp â‰ˆ 79k; cap just above */
        .sineAlignModPct    = 15,      /* ~4Ã— the 2810 (4Ã— R) to hold the heavy rotor */
        .sineRampModPct     = 20,      /* CURRENT-MATCHED to proven 2810 ramp (~12-13A):
                                        * 20/200=10% peak * 24V / 0.188Î© = 12.8A, safely under
                                        * ocSwLimit=16A. (Was 30 = 19A = guaranteed OC_SW trip.)
                                        * 2810 uses 5 into 0.05Î© = same ~12A. If the heavy Cobra
                                        * rotor STALLS in OL_RAMP raise toward 24-26 (keep Ia_pk
                                        * < 16A); if it OC_SW trips in OL_RAMP lower toward 16. */
        .zcDemagDutyThresh  = 45,
        .zcDemagBlankExtraPct = 18,    /* Ls unmeasured â€” near 2810; raise if early ZC miss */
        .ocLimitMa          = 20000,   /* CMP3 chop */
        .ocStartupMa        = 22000,
        .rampCurrentGateMa  = 12000,   /* heavy rotor draws more in accel; keep < ocLimitMa */
        TUNING_DEFAULTS,
    },
    [GSP_PROFILE_XROTOR] = {
        /* === Hobbywing XRotor 3110 1150KV (12N14P, 7PP, 4-6S, 88g) ===
         * Rs(phase-phase)=0.045Î©, KV=1150. SAME regime as the 2810 (profile 2):
         * low R, high KV, light rotor. This is the 2810 entry with only the
         * KV-driven fields changed (maxClosedLoopErpm + FOC Ke). Should bring up
         * almost exactly like the 2810 â€” start here. maxClosedLoopErpm anchored
         * just above the 1150KV theoretical ceiling; raise once it runs clean. */
        .rampTargetErpm     = 3000,
        .rampAccelErpmPerS  = 3000,    /* light rotor â€” same fast ramp as 2810 */
        .rampDutyPct        = 8,
        .clIdleDutyPct      = 6,
        .timingAdvMaxDeg    = 25,
        .hwzcCrossoverErpm  = 1500,
        .ocSwLimitMa        = 18000,
        .ocFaultMa          = 21000,
        .motorPolePairs     = 7,
        .alignDutyPct       = 3,
        .initialErpm        = 150,
        .maxClosedLoopErpm  = 210000,  /* 1150 * 24V * 7pp â‰ˆ 193k; cap just above */
        .sineAlignModPct    = 3,       /* same low-R regime as 2810 */
        .sineRampModPct     = 5,
        .zcDemagDutyThresh  = 40,
        .zcDemagBlankExtraPct = 20,
        .ocLimitMa          = 20000,
        .ocStartupMa        = 22000,
        .rampCurrentGateMa  = 10000,
        TUNING_DEFAULTS,
    },
    [GSP_PROFILE_VEX] = {
        /* === VEX 14mm micro (12N/6PP, 4000KV, 7.4V rated / 10V max) ===
         * Rs(pp)=0.44Î©, Ld/Lqâ‰ˆ18.4ÂµH(pp), no-load 0.65A, max-torque 7.25A,
         * stall 14A. Wizard-generated 2026-06-11, then BENCH-DERIVED overrides:
         * the 2810 hand-off-starvation experiment (rampTargetErpm=1200 â†’
         * 2/3 starts fiction-locked then OC'd, 1/3 ground through) proved a
         * 4000KV motor at the stock 3k hand-off has ~6 ADC counts of BEMF â€”
         * below the detection floor. 12k eRPM hand-off (only ~2k mech RPM)
         * restores the proven 2810-equivalent signal (~17+ counts); crossover
         * scaled to match. OC chain scaled to the motor (stall is only 14A â€”
         * the 24V-class 18/21A chain could never protect it). vbusUvAdc
         * lowered for the 10V supply (spec min 5V; UV â‰ˆ6.0V, derived startup
         * UV â‰ˆ4.8V). NOTE: phase dividers are 48V-scaled â€” BEMF lives in the
         * bottom ~435mV of the ADC at 10V; a divider rescale is the real
         * long-term fix (4.8x signal). */
        /* 2026-06-17 BENCH-PROVEN startup: ported from profile 2 (5010/2810).
         * The earlier "high hand-off" strategy (rampTarget=28k, crossover=24k,
         * align=14%, mod=28) was built on the theory that 4000KV BEMF is
         * undetectable below ~12k. That theory is RETRACTED â€” the real cause of
         * the half-speed/high-current was the ABS_FLOOR(Î») clamp using the wrong
         * motor's flux (583 vs 230). With focKeUvSRad=230 below, the low-hand-off
         * 2810 strategy runs the VEX clean: ALIGN/OL at ~2% duty, MORPH at 3k,
         * CL idle ~10.8k @ 0.3A, smooth climb. (run gui_auto_20260617_073714.) */
        .rampTargetErpm     = 3000,    /* low hand-off â€” BEMF is detectable here once the
                                        * Î» clamp is gone; no need to drag OL to 28k */
        .rampAccelErpmPerS  = 3000,    /* ~1s ramp 300->3000 */
        .rampDutyPct        = 8,       /* ramp duty cap (sine mod sets the real current) */
        .clIdleDutyPct      = 4,       /* clamps to MIN_DUTY -> ~10.8k idle @ Î»=230 */
        .timingAdvMaxDeg    = 20,      /* proven 2810 value; live-tune 0x22 if needed */
        .hwzcCrossoverErpm  = 1500,    /* engage HWZC right after morph */
        .ocSwLimitMa        = 9000,    /* SW soft backstop above the CMP3 chop */
        .ocFaultMa          = 11000,   /* SW hard fault (small motor: stall 14A) */
        .motorPolePairs     = 6,
        .alignDutyPct       = 3,       /* gentle align */
        .initialErpm        = 150,     /* gentle first steps */
        .maxClosedLoopErpm  = 240000,  /* runtime speed clamp = nominal no-load (4000KV x 10V x 6pp).
                                        * Field-weakening (advance) carries the rotor well past this:
                                        * bench reaches real closed-loop detection to ~285k, and rides
                                        * the clamp beyond. Live-tune 0x11 up to the descriptor max
                                        * (350000) to explore; 240k is a safe default. The advance
                                        * anchor is the compile-time RT_TIMING_ADV_FULL_ERPM, NOT this. */
        .sineAlignModPct    = 3,       /* low-mod align â€” R_LNâ‰ˆ0.22Î©, keep current ~3A */
        .sineRampModPct     = 5,       /* low-mod forced ramp */
        .zcDemagDutyThresh  = 40,
        .zcDemagBlankExtraPct = 20,    /* low-L motor: long demag tail */
        .ocLimitMa          = 600,     /* CMP3 chop ~6A bus â€” the real current bound for the
                                        * 7.25A-max-torque VEX (same shunt/board as 2810) */
        .ocStartupMa        = 22000,   /* startup relaxed (chop does the bounding) */
        .rampCurrentGateMa  = 10000,
        TUNING_DEFAULTS,
        /* 10V-supply override (TUNING default 500 â‰ˆ 9.3V leaves 0.7V margin
         * at a 10V bus â€” any sag faults). 320 â‰ˆ 6.0V; startup UV derives to
         * ~4.8V which matches the 5V spec minimum. */
        .vbusUvAdc          = 320,
    },
    [GSP_PROFILE_1407_2S] = {
        /* === 1407 4000KV 9N12P (6PP) @ 2S (7.4V nom / 8.4V max), FPV 3" ===
         * Based on the VEX 4000KV/6PP profile (same KV & pole count). Values are
         * physics/template ESTIMATES â€” bench-tune. Rs/Ls measured numbers wanted.
         * A2212 lessons baked in: PP=6 set right; CL-entry soft-start (global)
         * engages (idle 16% >> MIN_DUTY); per-profile falling-SW gate (config.h);
         * handoff chop ON (config.h OC_CLPCI_ENABLE=1) for the low-L startup. */
        .rampTargetErpm     = 12000,   /* high-KV: BEMF undetectable below ~12k (VEX lesson) */
        .rampAccelErpmPerS  = 8000,
        .rampDutyPct        = 25,
        .clIdleDutyPct      = 16,      /* ~32k eRPM idle at 8.4V; soft-start ramps into it */
        .timingAdvMaxDeg    = 22,      /* A2212 sweet spot ~23; start 22, tune at top */
        .hwzcCrossoverErpm  = 6000,
        .ocSwLimitMa        = 7250,
        .ocFaultMa          = 9500,
        .motorPolePairs     = 6,
        .alignDutyPct       = 25,
        .initialErpm        = 300,
        .maxClosedLoopErpm  = 202000,  /* 4000 * 8.4V * 6pp ~= 202k (under ~260k ceiling) */
        .sineAlignModPct    = 50,
        .sineRampModPct     = 50,
        .zcDemagDutyThresh  = 45,
        .zcDemagBlankExtraPct = 18,
        .ocLimitMa          = 9000,
        .ocStartupMa        = 10000,
        .rampCurrentGateMa  = 7000,
        TUNING_DEFAULTS,
        .vbusUvAdc          = 320,     /* ~6V (2S min ~6.6V) */
    },
    [GSP_PROFILE_1407_3S] = {
        /* === 1407 4000KV 9N12P (6PP) @ 3S (11.1V nom / 12.6V max), FPV 3" ===
         * Same motor as profile 7, scaled to 3S. No-load top ~302k EXCEEDS the
         * ~260k sensorless ceiling -> maxClosedLoopErpm CAPPED at 260000 (caps
         * before the top-end desync; a 3" prop won't free-spin there anyway).
         * Lower duties than 2S (more volts for the same current). ESTIMATES. */
        .rampTargetErpm     = 12000,
        .rampAccelErpmPerS  = 8000,
        .rampDutyPct        = 20,      /* 12.6V -> less duty than 2S for same current */
        .clIdleDutyPct      = 11,      /* ~33k eRPM idle at 12.6V */
        .timingAdvMaxDeg    = 22,
        .hwzcCrossoverErpm  = 6000,
        .ocSwLimitMa        = 7250,
        .ocFaultMa          = 9500,
        .motorPolePairs     = 6,
        .alignDutyPct       = 20,
        .initialErpm        = 300,
        .maxClosedLoopErpm  = 260000,  /* capped at sensorless ceiling (no-load 4000*12.6*6 ~= 302k) */
        .sineAlignModPct    = 40,      /* 12.6V: less mod than 2S for the same align current */
        .sineRampModPct     = 40,
        .zcDemagDutyThresh  = 45,
        .zcDemagBlankExtraPct = 18,
        .ocLimitMa          = 9000,
        .ocStartupMa        = 10000,
        .rampCurrentGateMa  = 7000,
        TUNING_DEFAULTS,
        .vbusUvAdc          = 480,     /* ~9V (3S min ~9.9V) */
    },
};

/* Max safe mA for OC params: DAC ceiling 4095 counts = 3299 mV */
#define OC_MAX_SAFE_MA  22000

/* â”€â”€ Descriptor table (31 entries) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

static const PARAM_DESCRIPTOR_T paramDescriptors[] = {
    /* Stage 1: Startup & Ramp (group 0) */
    { PARAM_ID_RAMP_TARGET_ERPM,      PARAM_TYPE_U16, PARAM_GROUP_STARTUP,    500,    20000, offsetof(GSP_PARAMS_T, rampTargetErpm),     2 },   /* max 10k->20k 2026-06-11: high-KV motors (VEX 4000KV) need a 12k+ hand-off for detectable BEMF */
    { PARAM_ID_RAMP_ACCEL_ERPM_PER_S, PARAM_TYPE_U16, PARAM_GROUP_STARTUP,     50,    20000, offsetof(GSP_PARAMS_T, rampAccelErpmPerS),  2 },   /* max 5k->20k 2026-06-11: 12k+ hand-offs need matching accel */
    { PARAM_ID_RAMP_DUTY_PCT,         PARAM_TYPE_U8,  PARAM_GROUP_STARTUP,      5,       80, offsetof(GSP_PARAMS_T, rampDutyPct),        1 },
    { PARAM_ID_ALIGN_DUTY_PCT,        PARAM_TYPE_U8,  PARAM_GROUP_STARTUP,      2,       50, offsetof(GSP_PARAMS_T, alignDutyPct),       1 },
    { PARAM_ID_INITIAL_ERPM,          PARAM_TYPE_U16, PARAM_GROUP_STARTUP,     50,     1000, offsetof(GSP_PARAMS_T, initialErpm),        2 },
    { PARAM_ID_SINE_ALIGN_MOD_PCT,    PARAM_TYPE_U8,  PARAM_GROUP_STARTUP,      2,       50, offsetof(GSP_PARAMS_T, sineAlignModPct),    1 },
    { PARAM_ID_SINE_RAMP_MOD_PCT,     PARAM_TYPE_U8,  PARAM_GROUP_STARTUP,      5,       80, offsetof(GSP_PARAMS_T, sineRampModPct),     1 },
    /* Stage 1: Closed-Loop Control (group 1) */
    { PARAM_ID_CL_IDLE_DUTY_PCT,      PARAM_TYPE_U8,  PARAM_GROUP_CLOSED_LOOP,  0,       30, offsetof(GSP_PARAMS_T, clIdleDutyPct),      1 },
    { PARAM_ID_TIMING_ADV_MAX_DEG,    PARAM_TYPE_U8,  PARAM_GROUP_CLOSED_LOOP,  0,       45, offsetof(GSP_PARAMS_T, timingAdvMaxDeg),     1 },
    { PARAM_ID_HWZC_CROSSOVER_ERPM,   PARAM_TYPE_U16, PARAM_GROUP_CLOSED_LOOP, 500,   20000, offsetof(GSP_PARAMS_T, hwzcCrossoverErpm),  2 },
    { PARAM_ID_MAX_CL_ERPM,           PARAM_TYPE_U32, PARAM_GROUP_CLOSED_LOOP, 5000, 350000, offsetof(GSP_PARAMS_T, maxClosedLoopErpm),  4 },  /* was 260000; raised so high-KV micros (e.g. VEX profile 6) can live-tune past the old cap and find their true field-weakened ceiling. This is the PI period-floor clamp + advance anchor; sensorless detection still works ~256k on the VEX (rej>0), so the wall was this descriptor, not the BEMF/sampling limit. */
    { PARAM_ID_ZC_DEMAG_DUTY_THRESH,  PARAM_TYPE_U8,  PARAM_GROUP_CLOSED_LOOP,  20,      90, offsetof(GSP_PARAMS_T, zcDemagDutyThresh),  1 },
    { PARAM_ID_ZC_DEMAG_BLANK_EXTRA,  PARAM_TYPE_U8,  PARAM_GROUP_CLOSED_LOOP,   0,      30, offsetof(GSP_PARAMS_T, zcDemagBlankExtraPct), 1 },
    /* Current Protection (group 2) */
    { PARAM_ID_OC_SW_LIMIT_MA,        PARAM_TYPE_U16, PARAM_GROUP_OVERCURRENT,  500, OC_MAX_SAFE_MA, offsetof(GSP_PARAMS_T, ocSwLimitMa),      2 },
    { PARAM_ID_OC_FAULT_MA,           PARAM_TYPE_U16, PARAM_GROUP_OVERCURRENT, 1000, OC_MAX_SAFE_MA, offsetof(GSP_PARAMS_T, ocFaultMa),        2 },
    { PARAM_ID_OC_LIMIT_MA,           PARAM_TYPE_U16, PARAM_GROUP_OVERCURRENT,  501, OC_MAX_SAFE_MA, offsetof(GSP_PARAMS_T, ocLimitMa),        2 },
    { PARAM_ID_OC_STARTUP_MA,         PARAM_TYPE_U16, PARAM_GROUP_OVERCURRENT, 5000, OC_MAX_SAFE_MA, offsetof(GSP_PARAMS_T, ocStartupMa),      2 },
    { PARAM_ID_RAMP_CURRENT_GATE_MA,  PARAM_TYPE_U16, PARAM_GROUP_OVERCURRENT,    0, OC_MAX_SAFE_MA, offsetof(GSP_PARAMS_T, rampCurrentGateMa), 2 },
    /* ZC Detection (group 3) */
    { PARAM_ID_ZC_BLANKING_PCT,       PARAM_TYPE_U8,  PARAM_GROUP_ZC_DETECT,    1,    15, offsetof(GSP_PARAMS_T, zcBlankingPercent),   1 },
    { PARAM_ID_ZC_ADC_DEADBAND,       PARAM_TYPE_U8,  PARAM_GROUP_ZC_DETECT,    0,    20, offsetof(GSP_PARAMS_T, zcAdcDeadband),       1 },
    { PARAM_ID_ZC_SYNC_THRESHOLD,     PARAM_TYPE_U8,  PARAM_GROUP_ZC_DETECT,    4,    20, offsetof(GSP_PARAMS_T, zcSyncThreshold),     1 },
    { PARAM_ID_ZC_FILTER_THRESHOLD,   PARAM_TYPE_U8,  PARAM_GROUP_ZC_DETECT,    1,    10, offsetof(GSP_PARAMS_T, zcFilterThreshold),   1 },
    /* Duty Slew (group 4) */
    { PARAM_ID_DUTY_SLEW_UP,          PARAM_TYPE_U8,  PARAM_GROUP_DUTY_SLEW,   1,    20, offsetof(GSP_PARAMS_T, dutySlewUpPctPerMs),  1 },
    { PARAM_ID_DUTY_SLEW_DOWN,        PARAM_TYPE_U8,  PARAM_GROUP_DUTY_SLEW,   1,    50, offsetof(GSP_PARAMS_T, dutySlewDownPctPerMs), 1 },
    { PARAM_ID_POST_SYNC_SETTLE_MS,   PARAM_TYPE_U16, PARAM_GROUP_DUTY_SLEW, 100,  5000, offsetof(GSP_PARAMS_T, postSyncSettleMs),    2 },
    { PARAM_ID_POST_SYNC_SLEW_DIV,    PARAM_TYPE_U8,  PARAM_GROUP_DUTY_SLEW,   1,    16, offsetof(GSP_PARAMS_T, postSyncSlewDivisor), 1 },
    /* Voltage Protection (group 5) */
    { PARAM_ID_VBUS_OV_ADC,           PARAM_TYPE_U16, PARAM_GROUP_VOLTAGE,   2000,  4000, offsetof(GSP_PARAMS_T, vbusOvAdc),          2 },
    { PARAM_ID_VBUS_UV_ADC,           PARAM_TYPE_U16, PARAM_GROUP_VOLTAGE,    200,  2000, offsetof(GSP_PARAMS_T, vbusUvAdc),          2 },
    /* Recovery (group 6) */
    { PARAM_ID_DESYNC_COAST_MS,       PARAM_TYPE_U16, PARAM_GROUP_RECOVERY,    50,  1000, offsetof(GSP_PARAMS_T, desyncCoastMs),      2 },
    { PARAM_ID_DESYNC_MAX_RESTARTS,   PARAM_TYPE_U8,  PARAM_GROUP_RECOVERY,     0,    10, offsetof(GSP_PARAMS_T, desyncMaxRestarts),  1 },
    /* Morphâ†’CL lock gate (group 0 = startup) */
    { PARAM_ID_MORPH_LOCK_ZC_COUNT,   PARAM_TYPE_U8,  PARAM_GROUP_STARTUP,      2,    20, offsetof(GSP_PARAMS_T, morphLockZcCount),   1 },
    { PARAM_ID_MORPH_LOCK_TOL_PCT,    PARAM_TYPE_U8,  PARAM_GROUP_STARTUP,      5,    60, offsetof(GSP_PARAMS_T, morphLockTolPct),    1 },
    /* I-f spin-up (group 0 = startup) */
    { PARAM_ID_IF_CURRENT_CA,         PARAM_TYPE_U16, PARAM_GROUP_STARTUP,    100,  2000, offsetof(GSP_PARAMS_T, ifCurrentCa),        2 },
    { PARAM_ID_IF_RAMP_ERPM_PER_S,    PARAM_TYPE_U16, PARAM_GROUP_STARTUP,   1000, 60000, offsetof(GSP_PARAMS_T, ifRampErpmPerS),     2 },
    /* Motor Hardware (group 7) */
    { PARAM_ID_MOTOR_POLE_PAIRS,      PARAM_TYPE_U8,  PARAM_GROUP_MOTOR_HW,    1,    20, offsetof(GSP_PARAMS_T, motorPolePairs),     1 },
};

#define PARAM_COUNT (sizeof(paramDescriptors) / sizeof(paramDescriptors[0]))

/* â”€â”€ Helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

static const PARAM_DESCRIPTOR_T *FindDescriptor(uint16_t id)
{
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
        if (paramDescriptors[i].id == id)
            return &paramDescriptors[i];
    }
    return NULL;
}

static uint32_t ReadField(const PARAM_DESCRIPTOR_T *desc)
{
    const uint8_t *base = (const uint8_t *)&gspParams;
    if (desc->fieldSize == 1)
        return *(const uint8_t *)(base + desc->offsetInParams);
    else if (desc->fieldSize == 2) {
        uint16_t v;
        memcpy(&v, base + desc->offsetInParams, 2);
        return v;
    } else {
        uint32_t v;
        memcpy(&v, base + desc->offsetInParams, 4);
        return v;
    }
}

static void WriteField(const PARAM_DESCRIPTOR_T *desc, uint32_t value)
{
    uint8_t *base = (uint8_t *)&gspParams;
    if (desc->fieldSize == 1) {
        uint8_t v = (uint8_t)value;
        *(base + desc->offsetInParams) = v;
    } else if (desc->fieldSize == 2) {
        uint16_t v = (uint16_t)value;
        memcpy(base + desc->offsetInParams, &v, 2);
    } else {
        memcpy(base + desc->offsetInParams, &value, 4);
    }
}

/* â”€â”€ OC mA to ADC conversion â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

static uint16_t OcMaToAdcCounts(uint16_t ma)
{
#if FEATURE_HW_OVERCURRENT
    uint32_t tripMv = OC_VREF_MV +
        ((uint32_t)ma * OC_SHUNT_MOHM * OC_GAIN_X100 / 100000);
    return (uint16_t)((uint32_t)tripMv * 4096 / OC_VADC_MV);
#else
    (void)ma;
    return 0;
#endif
}

/* â”€â”€ Public API â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

void GSP_ParamsInitDefaults(void)
{
    /* Load from compile-time MOTOR_PROFILE */
    activeProfile = MOTOR_PROFILE;

#if MOTOR_PROFILE < GSP_PROFILE_COUNT
    memcpy(&gspParams, &profileDefaults[MOTOR_PROFILE], sizeof(gspParams));
#else
    /* Custom profile at compile time â€” use A2212 as base */
    memcpy(&gspParams, &profileDefaults[GSP_PROFILE_A2212], sizeof(gspParams));
#endif

    /* Override from compile-time config for features that may be disabled */
#if !FEATURE_TIMING_ADVANCE
    gspParams.timingAdvMaxDeg = 0;
#endif
#if !FEATURE_ADC_CMP_ZC
    gspParams.hwzcCrossoverErpm = 0;
#endif
#if !FEATURE_HW_OVERCURRENT
    gspParams.ocSwLimitMa = 0;
    gspParams.ocFaultMa = 0;
    gspParams.ocLimitMa = 0;
    gspParams.ocStartupMa = 0;
    gspParams.rampCurrentGateMa = 0;
#endif

}

void GSP_RecomputeDerived(void)
{
    GSP_PARAMS_T *p = &gspParams;
    GSP_DERIVED_T *d = &gspDerived;

    /* Ramp duty cap in PWM counts */
    d->rampDutyCap = (uint32_t)(p->rampDutyPct / 100.0f * LOOPTIME_TCY);

    /* CL idle duty floor */
    if (p->clIdleDutyPct > 0)
        d->clIdleDuty = (uint32_t)(p->clIdleDutyPct / 100.0f * LOOPTIME_TCY);
    else
        d->clIdleDuty = MIN_DUTY;

    /* Sine eRPM ramp rate (Q16 fractional per Timer1 tick) */
    d->sineErpmRampRateQ16 =
        (uint32_t)((uint64_t)p->rampAccelErpmPerS * 65536UL / 10000UL);

    /* Min step period from ramp target eRPM (Timer1 ticks) */
    if (p->rampTargetErpm > 0) {
        d->minStepPeriod = (uint16_t)(100000UL / p->rampTargetErpm);
        if (d->minStepPeriod < 1) d->minStepPeriod = 1;
    } else {
        d->minStepPeriod = 1;
    }

    /* Convert to ADC ISR ticks: ratio = PWMFREQUENCY_HZ/10000 (4.5 at 45 kHz).
     * FIXED 2026-06-11: was *12/5, the stale 24 kHz factor. */
#if FEATURE_BEMF_CLOSED_LOOP
    d->minAdcStepPeriod = (uint16_t)(((uint32_t)d->minStepPeriod * PWMFREQUENCY_HZ) / 10000UL);
    if (d->minAdcStepPeriod < 1) d->minAdcStepPeriod = 1;
#else
    d->minAdcStepPeriod = 1;
#endif

    /* OC thresholds: mA â†’ ADC counts */
    d->ocSwLimitAdc  = OcMaToAdcCounts(p->ocSwLimitMa);
    d->ocFaultAdcVal = OcMaToAdcCounts(p->ocFaultMa);

    /* â”€â”€ New derived values (Phase 1.5) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

    /* Align duty in PWM counts */
    d->alignDuty = (uint32_t)(p->alignDutyPct / 100.0f * LOOPTIME_TCY);

    /* Initial step period from initial eRPM */
    if (p->initialErpm > 0) {
        d->initialStepPeriod = (uint16_t)(100000UL / p->initialErpm);
        if (d->initialStepPeriod < 1) d->initialStepPeriod = 1;
    } else {
        d->initialStepPeriod = 1;
    }

    /* Initial ADC step period */
#if FEATURE_BEMF_CLOSED_LOOP
    d->initialAdcStepPeriod = (uint16_t)(((uint32_t)d->initialStepPeriod * PWMFREQUENCY_HZ) / 10000UL);
    if (d->initialAdcStepPeriod < 1) d->initialAdcStepPeriod = 1;
#else
    d->initialAdcStepPeriod = 1;
#endif

    /* Minimum CL ADC step period from max closed-loop eRPM */
#if FEATURE_BEMF_CLOSED_LOOP
    if (p->maxClosedLoopErpm > 0) {
        uint32_t maxClStepT1 = 100000UL / p->maxClosedLoopErpm;
        if (maxClStepT1 < 1) maxClStepT1 = 1;
        d->minClAdcStepPeriod = (uint16_t)(((uint32_t)maxClStepT1 * PWMFREQUENCY_HZ) / 10000UL);
        if (d->minClAdcStepPeriod < 1) d->minClAdcStepPeriod = 1;
    } else {
        d->minClAdcStepPeriod = 1;
    }
#else
    d->minClAdcStepPeriod = 1;
#endif

    /* Sine amplitude limits in PWM counts */
#if FEATURE_SINE_STARTUP
    d->sineMinAmplitude = (uint32_t)(LOOPTIME_TCY * p->sineAlignModPct / 200);
    d->sineMaxAmplitude = (uint32_t)(LOOPTIME_TCY * p->sineRampModPct / 200);
#else
    d->sineMinAmplitude = 0;
    d->sineMaxAmplitude = 0;
#endif

    /* Duty slew rates (per ADC ISR tick) */
#if FEATURE_DUTY_SLEW
    d->dutySlewUpRate = (uint32_t)(
        (uint64_t)MAX_DUTY * p->dutySlewUpPctPerMs / 100
        / (PWMFREQUENCY_HZ / 1000));
    d->dutySlewDownRate = (uint32_t)(
        (uint64_t)MAX_DUTY * p->dutySlewDownPctPerMs / 100
        / (PWMFREQUENCY_HZ / 1000));
    d->postSyncSettleTicks = (uint16_t)(
        (uint32_t)p->postSyncSettleMs * PWMFREQUENCY_HZ / 1000);
#else
    d->dutySlewUpRate = 0;
    d->dutySlewDownRate = 0;
    d->postSyncSettleTicks = 0;
#endif

    /* OC CMP3 thresholds with hardware safety clamp */
#if FEATURE_HW_OVERCURRENT
    d->ocCmp3DacVal = OcMaToAdcCounts(p->ocLimitMa);
    if (d->ocCmp3DacVal >= 4096) d->ocCmp3DacVal = 4095;

    d->ocCmp3StartupDac = OcMaToAdcCounts(p->ocStartupMa);
    if (d->ocCmp3StartupDac >= 4096) d->ocCmp3StartupDac = 4095;

    if (p->rampCurrentGateMa > 0)
        d->rampCurrentGateAdc = OcMaToAdcCounts(p->rampCurrentGateMa);
    else
        d->rampCurrentGateAdc = 0;
#else
    d->ocCmp3DacVal = 0;
    d->ocCmp3StartupDac = 0;
    d->rampCurrentGateAdc = 0;
#endif

    /* Desync coast-down counts (Timer1 = 100us ticks) */
#if FEATURE_DESYNC_RECOVERY
    d->desyncCoastCounts = (uint32_t)(p->desyncCoastMs * 10);
#else
    d->desyncCoastCounts = 0;
#endif

    /* HWZC step period limits + noise floor clamping */
#if FEATURE_ADC_CMP_ZC
    if (p->maxClosedLoopErpm > 0)
        d->hwzcMinStepTicks = 1000000000UL / p->maxClosedLoopErpm;
    else
        d->hwzcMinStepTicks = 1000000000UL;

    d->hwzcNoiseFloorTicks = d->hwzcMinStepTicks * 2 / 3;
    /* Fix 1: clamp noise floor to PWM noise interval + 20% margin */
    {
        uint32_t pwmNoiseInterval = HWZC_TIMER_HZ / PWMFREQUENCY_HZ; /* 4167 */
        uint32_t minFloor = pwmNoiseInterval + pwmNoiseInterval / 5;  /* +20% */
        if (d->hwzcNoiseFloorTicks < minFloor)
            d->hwzcNoiseFloorTicks = minFloor;
    }
#else
    d->hwzcMinStepTicks = 0;
    d->hwzcNoiseFloorTicks = 0;
#endif

    /* Relaxed UV threshold during pre-sync startup (80% of normal UV) */
    d->vbusUvStartupAdc = (uint16_t)(p->vbusUvAdc * 4 / 5);
    if (d->vbusUvStartupAdc < 200) d->vbusUvStartupAdc = 200;
}

/* â”€â”€ Cross-parameter validation (bilateral, 10 checks) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

static PARAM_RESULT_T CrossValidate(uint16_t id, uint32_t value)
{
    GSP_PARAMS_T *p = &gspParams;

    switch (id) {
    /* rampTargetErpm > initialErpm (bilateral) */
    case PARAM_ID_RAMP_TARGET_ERPM:
        if (value <= p->initialErpm)
            return PARAM_ERR_CROSS_VALIDATION;
        if (value >= p->maxClosedLoopErpm)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_INITIAL_ERPM:
        if (value >= p->rampTargetErpm)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    /* maxClosedLoopErpm > rampTargetErpm (bilateral) */
    case PARAM_ID_MAX_CL_ERPM:
        if (value <= p->rampTargetErpm)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    /* zcSyncThreshold >= MORPH_ZC_THRESHOLD (4) */
    case PARAM_ID_ZC_SYNC_THRESHOLD:
#if FEATURE_SINE_STARTUP
        if (value < MORPH_ZC_THRESHOLD)
            return PARAM_ERR_CROSS_VALIDATION;
#endif
        if (value <= p->zcFilterThreshold)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    /* zcFilterThreshold < zcSyncThreshold (bilateral) */
    case PARAM_ID_ZC_FILTER_THRESHOLD:
        if (value >= p->zcSyncThreshold)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    /* OC chain: ocSwLimitMa < ocLimitMa <= ocFaultMa */
    case PARAM_ID_OC_SW_LIMIT_MA:
        if (value >= p->ocLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        if (value > p->ocFaultMa)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_OC_LIMIT_MA:
        if (value <= p->ocSwLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        if (value > p->ocFaultMa)
            return PARAM_ERR_CROSS_VALIDATION;
        /* ocStartupMa >= ocLimitMa */
        if (p->ocStartupMa < value)
            return PARAM_ERR_CROSS_VALIDATION;
        /* rampCurrentGateMa < ocLimitMa (if nonzero) */
        if (p->rampCurrentGateMa != 0 && p->rampCurrentGateMa >= value)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_OC_FAULT_MA:
        if (value < p->ocLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        if (value < p->ocSwLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_OC_STARTUP_MA:
        if (value < p->ocLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_RAMP_CURRENT_GATE_MA:
        if (value != 0 && value >= p->ocLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    /* vbusOvAdc > vbusUvAdc (bilateral) */
    case PARAM_ID_VBUS_OV_ADC:
        if (value <= p->vbusUvAdc)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_VBUS_UV_ADC:
        if (value >= p->vbusOvAdc)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    default:
        break;
    }

    return PARAM_OK;
}

PARAM_RESULT_T GSP_ParamSet(uint16_t id, uint32_t value)
{
    const PARAM_DESCRIPTOR_T *desc = FindDescriptor(id);
    if (desc == NULL)
        return PARAM_ERR_UNKNOWN_ID;

    /* Bounds check */
    if (value < desc->minVal || value > desc->maxVal)
        return PARAM_ERR_OUT_OF_RANGE;

    /* Cross-parameter validation */
    PARAM_RESULT_T cv = CrossValidate(id, value);
    if (cv != PARAM_OK)
        return cv;

    /* Write field and recompute derived */
    WriteField(desc, value);
    GSP_RecomputeDerived();

    /* Signal FOC re-init if a FOC param was changed */

    return PARAM_OK;
}

bool GSP_ParamGet(uint16_t id, uint32_t *out)
{
    const PARAM_DESCRIPTOR_T *desc = FindDescriptor(id);
    if (desc == NULL)
        return false;

    *out = ReadField(desc);
    return true;
}

uint8_t GSP_ParamGetCount(void)
{
    return (uint8_t)PARAM_COUNT;
}

const PARAM_DESCRIPTOR_T *GSP_ParamGetDescriptor(uint8_t idx)
{
    if (idx >= PARAM_COUNT)
        return NULL;
    return &paramDescriptors[idx];
}

/* â”€â”€ Profile management â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

bool GSP_ParamsLoadProfile(uint8_t profileId)
{
    if (profileId < GSP_PROFILE_COUNT) {
        /* Built-in profile: copy defaults */
        memcpy(&gspParams, &profileDefaults[profileId], sizeof(gspParams));
        activeProfile = profileId;
        GSP_RecomputeDerived();
        /* Signal FOC re-init needed (checked by ISR when IDLE) */
        return true;
    } else if (profileId == GSP_PROFILE_CUSTOM) {
        /* Custom: adopt current values as-is, just mark profile */
        activeProfile = GSP_PROFILE_CUSTOM;
        return true;
    }
    return false;
}

uint8_t GSP_ParamsGetActiveProfile(void)
{
    return activeProfile;
}

/* â”€â”€ EEPROM persistence (V2/V3) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */

#define GSP_PERSIST_OFFSET  16  /* Byte offset within GARUDA_CONFIG_T.reserved */

/**
 * Clamp all gspParams fields to descriptor bounds, then verify
 * cross-parameter invariants.  Returns true if all invariants hold
 * (possibly after clamping), false if invariants are violated
 * and the caller should fall back to profile defaults.
 */
#if FEATURE_GSP_EEPROM   /* EEPROM-only helpers: compiled only for production builds */
static bool SanitizeLoadedParams(void)
{
    /* Validate activeProfile */
    if (activeProfile > GSP_PROFILE_CUSTOM)
        return false;

    /* Pass 1: clamp every field to its descriptor [min, max] */
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
        const PARAM_DESCRIPTOR_T *d = &paramDescriptors[i];
        uint32_t v = ReadField(d);
        if (v < d->minVal)      { WriteField(d, d->minVal); }
        else if (v > d->maxVal) { WriteField(d, d->maxVal); }
    }

    /* Pass 2: verify cross-parameter invariants.
     * If any fails, the entire param set is suspect. */
    const GSP_PARAMS_T *p = &gspParams;

    if (p->rampTargetErpm <= p->initialErpm)            return false;
    if (p->maxClosedLoopErpm <= p->rampTargetErpm)      return false;
#if FEATURE_SINE_STARTUP
    if (p->zcSyncThreshold < MORPH_ZC_THRESHOLD)        return false;
#endif
    if (p->zcFilterThreshold >= p->zcSyncThreshold)     return false;
    if (p->ocSwLimitMa >= p->ocLimitMa)                 return false;
    if (p->ocLimitMa > p->ocFaultMa)                    return false;
    if (p->ocStartupMa < p->ocLimitMa)                  return false;
    if (p->rampCurrentGateMa != 0 &&
        p->rampCurrentGateMa >= p->ocLimitMa)           return false;
    if (p->vbusOvAdc <= p->vbusUvAdc)                   return false;

    return true;
}

/** Fall back to profile defaults when sanitization fails. */
static void FallbackToProfileDefaults(void)
{
    uint8_t fallback = (activeProfile < GSP_PROFILE_COUNT)
                        ? activeProfile : (uint8_t)MOTOR_PROFILE;
    if (fallback >= GSP_PROFILE_COUNT)
        fallback = GSP_PROFILE_A2212;
    memcpy(&gspParams, &profileDefaults[fallback], sizeof(gspParams));
    activeProfile = fallback;
}

/* â”€â”€ Defaults signature (Tier 3 auto-invalidation) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
 * A CRC16 over this build's profileDefaults[MOTOR_PROFILE] is stored in EEPROM
 * on save and re-checked on load. If a developer edits that profile's compiled
 * defaults (or changes MOTOR_PROFILE) and reflashes, the signature no longer
 * matches â†’ the EEPROM overlay is rejected and the fresh compiled defaults win,
 * with no manual marker bump or NVM reset. Live GSP tuning still persists across
 * power cycles for an unchanged build (the saved values match the signature).
 *
 * Stored at a fixed offset in GARUDA_CONFIG_T.reserved, clear of the 82-byte V3
 * persist struct (offsets 16..97). Reserved spans 16..127, so 100 is free. */
#define GSP_DEFAULTS_SIG_OFFSET  100

/* CRC16-CCITT (poly 0x1021, init 0xFFFF). Local to this file; only needs to be
 * stable within a single build, not match the GSP framing CRC. */
static uint16_t GSP_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

/* Signature of this firmware's compiled defaults for the active build profile. */
static uint16_t GSP_DefaultsSignature(void)
{
    uint8_t idx = (MOTOR_PROFILE < GSP_PROFILE_COUNT)
                  ? (uint8_t)MOTOR_PROFILE : (uint8_t)GSP_PROFILE_A2212;
    return GSP_Crc16((const uint8_t *)&profileDefaults[idx],
                     (uint16_t)sizeof(profileDefaults[0]));
}
#endif /* FEATURE_GSP_EEPROM (helpers) */

void GSP_ParamsLoadFromConfig(const void *cfg)
{
#if !FEATURE_GSP_EEPROM
    /* DEVELOPMENT (FEATURE_GSP_EEPROM=0): code is the single source of truth.
     * Ignore the EEPROM overlay entirely so gsp_params.c profileDefaults[] always
     * win and no stale EEPROM can shadow a code edit. */
    (void)cfg;
    return;
#else
    const uint8_t *base = (const uint8_t *)cfg;

    /* Tier 3: reject stale EEPROM whose stored defaults-signature does not match
     * this build's compiled defaults. Blank/old EEPROM (no signature) also fails
     * this check â†’ compiled defaults are kept. After the next save, the correct
     * signature is written and same-build tuning persists normally. */
    uint16_t storedSig;
    memcpy(&storedSig, base + GSP_DEFAULTS_SIG_OFFSET, sizeof(storedSig));
    if (storedSig != GSP_DefaultsSignature())
        return;  /* keep compile-time defaults from GSP_ParamsInitDefaults() */

    uint8_t marker;
    memcpy(&marker, base + GSP_PERSIST_OFFSET, 1);

    if (marker == GSP_PERSIST_V3_MARKER) {
        /* V3 schema: V2 fields + 17 FOC params */
        GSP_CONFIG_PERSIST_V3_T persist;
        memcpy(&persist, base + GSP_PERSIST_OFFSET, sizeof(persist));

        activeProfile                   = persist.activeProfile;
        gspParams.rampDutyPct           = persist.rampDutyPct;
        gspParams.clIdleDutyPct         = persist.clIdleDutyPct;
        gspParams.timingAdvMaxDeg       = persist.timingAdvMaxDeg;
        gspParams.rampTargetErpm        = persist.rampTargetErpm;
        gspParams.rampAccelErpmPerS     = persist.rampAccelErpmPerS;
        gspParams.hwzcCrossoverErpm     = persist.hwzcCrossoverErpm;
        gspParams.ocSwLimitMa           = persist.ocSwLimitMa;
        gspParams.ocFaultMa             = persist.ocFaultMa;
        gspParams.motorPolePairs        = persist.motorPolePairs;
        gspParams.alignDutyPct          = persist.alignDutyPct;
        gspParams.initialErpm           = persist.initialErpm;
        gspParams.sineAlignModPct       = persist.sineAlignModPct;
        gspParams.sineRampModPct        = persist.sineRampModPct;
        gspParams.zcDemagDutyThresh     = persist.zcDemagDutyThresh;
        gspParams.zcDemagBlankExtraPct  = persist.zcDemagBlankExtraPct;
        gspParams.ocLimitMa             = persist.ocLimitMa;
        gspParams.ocStartupMa           = persist.ocStartupMa;
        gspParams.rampCurrentGateMa     = persist.rampCurrentGateMa;
        gspParams.maxClosedLoopErpm     = ((uint32_t)persist.maxClErpmHi << 16) |
                                           persist.maxClosedLoopErpmLo;
        gspParams.dutySlewUpPctPerMs    = persist.dutySlewUpPctPerMs;
        gspParams.dutySlewDownPctPerMs  = persist.dutySlewDownPctPerMs;
        gspParams.postSyncSettleMs      = persist.postSyncSettleMs;
        gspParams.postSyncSlewDivisor   = persist.postSyncSlewDivisor;
        gspParams.zcBlankingPercent     = persist.zcBlankingPercent;
        gspParams.zcAdcDeadband         = persist.zcAdcDeadband;
        gspParams.zcSyncThreshold       = persist.zcSyncThreshold;
        gspParams.zcFilterThreshold     = persist.zcFilterThreshold;
        gspParams.vbusOvAdc             = persist.vbusOvAdc;
        gspParams.vbusUvAdc             = persist.vbusUvAdc;
        gspParams.desyncCoastMs         = persist.desyncCoastMs;
        gspParams.desyncMaxRestarts     = persist.desyncMaxRestarts;
        /* V3 FOC fields */

        if (!SanitizeLoadedParams())
            FallbackToProfileDefaults();

    } else if (marker == GSP_PERSIST_V2_MARKER) {
        /* V2 schema: load 31 6-step params, FOC params get profile defaults */
        GSP_CONFIG_PERSIST_V2_T persist;
        memcpy(&persist, base + GSP_PERSIST_OFFSET, sizeof(persist));

        activeProfile                   = persist.activeProfile;
        gspParams.rampDutyPct           = persist.rampDutyPct;
        gspParams.clIdleDutyPct         = persist.clIdleDutyPct;
        gspParams.timingAdvMaxDeg       = persist.timingAdvMaxDeg;
        gspParams.rampTargetErpm        = persist.rampTargetErpm;
        gspParams.rampAccelErpmPerS     = persist.rampAccelErpmPerS;
        gspParams.hwzcCrossoverErpm     = persist.hwzcCrossoverErpm;
        gspParams.ocSwLimitMa           = persist.ocSwLimitMa;
        gspParams.ocFaultMa             = persist.ocFaultMa;
        gspParams.motorPolePairs        = persist.motorPolePairs;
        gspParams.alignDutyPct          = persist.alignDutyPct;
        gspParams.initialErpm           = persist.initialErpm;
        gspParams.sineAlignModPct       = persist.sineAlignModPct;
        gspParams.sineRampModPct        = persist.sineRampModPct;
        gspParams.zcDemagDutyThresh     = persist.zcDemagDutyThresh;
        gspParams.zcDemagBlankExtraPct  = persist.zcDemagBlankExtraPct;
        gspParams.ocLimitMa             = persist.ocLimitMa;
        gspParams.ocStartupMa           = persist.ocStartupMa;
        gspParams.rampCurrentGateMa     = persist.rampCurrentGateMa;
        gspParams.maxClosedLoopErpm     = ((uint32_t)persist.maxClErpmHi << 16) |
                                           persist.maxClosedLoopErpmLo;
        gspParams.dutySlewUpPctPerMs    = persist.dutySlewUpPctPerMs;
        gspParams.dutySlewDownPctPerMs  = persist.dutySlewDownPctPerMs;
        gspParams.postSyncSettleMs      = persist.postSyncSettleMs;
        gspParams.postSyncSlewDivisor   = persist.postSyncSlewDivisor;
        gspParams.zcBlankingPercent     = persist.zcBlankingPercent;
        gspParams.zcAdcDeadband         = persist.zcAdcDeadband;
        gspParams.zcSyncThreshold       = persist.zcSyncThreshold;
        gspParams.zcFilterThreshold     = persist.zcFilterThreshold;
        gspParams.vbusOvAdc             = persist.vbusOvAdc;
        gspParams.vbusUvAdc             = persist.vbusUvAdc;
        gspParams.desyncCoastMs         = persist.desyncCoastMs;
        gspParams.desyncMaxRestarts     = persist.desyncMaxRestarts;
        /* FOC params keep their profile defaults from InitDefaults() */

        if (!SanitizeLoadedParams())
            FallbackToProfileDefaults();

    } else if (marker == GSP_PERSIST_V1_MARKER) {
        /* V1 schema: load 8 Stage 1 params, keep profile defaults for rest.
         * activeProfile = MOTOR_PROFILE (compile-time, V1 migration rule). */
        GSP_CONFIG_PERSIST_V1_T persist;
        memcpy(&persist, base + GSP_PERSIST_OFFSET, sizeof(persist));

        gspParams.rampDutyPct       = persist.rampDutyPct;
        gspParams.clIdleDutyPct     = persist.clIdleDutyPct;
        gspParams.timingAdvMaxDeg   = persist.timingAdvMaxDeg;
        gspParams.rampTargetErpm    = persist.rampTargetErpm;
        gspParams.rampAccelErpmPerS = persist.rampAccelErpmPerS;
        gspParams.hwzcCrossoverErpm = persist.hwzcCrossoverErpm;
        gspParams.ocSwLimitMa       = persist.ocSwLimitMa;
        gspParams.ocFaultMa         = persist.ocFaultMa;
        /* 23 new params keep their profile defaults from InitDefaults() */

        /* Sanitize: clamp to bounds + check cross-parameter invariants */
        if (!SanitizeLoadedParams())
            FallbackToProfileDefaults();
    }
    /* else: unknown marker â€” keep compile-time defaults */
#endif /* FEATURE_GSP_EEPROM */
}

void GSP_ParamsSaveToConfig(void *cfg)
{
#if !FEATURE_GSP_EEPROM
    /* DEVELOPMENT (FEATURE_GSP_EEPROM=0): never persist to NVM. Live GSP tuning
     * stays RAM-only and reverts to the compiled defaults on the next power cycle,
     * so no stale state can accumulate in EEPROM. */
    (void)cfg;
    return;
#else
    uint8_t *base = (uint8_t *)cfg;
    GSP_CONFIG_PERSIST_V3_T persist;
    memset(&persist, 0, sizeof(persist));

    persist.schemaMarker           = GSP_PERSIST_V3_MARKER;
    persist.activeProfile          = activeProfile;
    persist.rampDutyPct            = gspParams.rampDutyPct;
    persist.clIdleDutyPct          = gspParams.clIdleDutyPct;
    persist.timingAdvMaxDeg        = gspParams.timingAdvMaxDeg;
    persist.rampTargetErpm         = gspParams.rampTargetErpm;
    persist.rampAccelErpmPerS      = gspParams.rampAccelErpmPerS;
    persist.hwzcCrossoverErpm      = gspParams.hwzcCrossoverErpm;
    persist.ocSwLimitMa            = gspParams.ocSwLimitMa;
    persist.ocFaultMa              = gspParams.ocFaultMa;
    persist.motorPolePairs         = gspParams.motorPolePairs;
    persist.alignDutyPct           = gspParams.alignDutyPct;
    persist.initialErpm            = gspParams.initialErpm;
    persist.sineAlignModPct        = gspParams.sineAlignModPct;
    persist.sineRampModPct         = gspParams.sineRampModPct;
    persist.zcDemagDutyThresh      = gspParams.zcDemagDutyThresh;
    persist.zcDemagBlankExtraPct   = gspParams.zcDemagBlankExtraPct;
    persist.ocLimitMa              = gspParams.ocLimitMa;
    persist.ocStartupMa            = gspParams.ocStartupMa;
    persist.rampCurrentGateMa      = gspParams.rampCurrentGateMa;
    persist.maxClosedLoopErpmLo    = (uint16_t)(gspParams.maxClosedLoopErpm & 0xFFFF);
    persist.maxClErpmHi            = (uint8_t)((gspParams.maxClosedLoopErpm >> 16) & 0xFF);
    persist.dutySlewUpPctPerMs     = gspParams.dutySlewUpPctPerMs;
    persist.dutySlewDownPctPerMs   = gspParams.dutySlewDownPctPerMs;
    persist.postSyncSettleMs       = gspParams.postSyncSettleMs;
    persist.postSyncSlewDivisor    = gspParams.postSyncSlewDivisor;
    persist.zcBlankingPercent      = gspParams.zcBlankingPercent;
    persist.zcAdcDeadband          = gspParams.zcAdcDeadband;
    persist.zcSyncThreshold        = gspParams.zcSyncThreshold;
    persist.zcFilterThreshold      = gspParams.zcFilterThreshold;
    persist.vbusOvAdc              = gspParams.vbusOvAdc;
    persist.vbusUvAdc              = gspParams.vbusUvAdc;
    persist.desyncCoastMs          = gspParams.desyncCoastMs;
    persist.desyncMaxRestarts      = gspParams.desyncMaxRestarts;
    /* V3 FOC fields */

    memcpy(base + GSP_PERSIST_OFFSET, &persist, sizeof(persist));

    /* Tier 3: stamp this build's defaults-signature so the saved values are
     * accepted on reload only while profileDefaults[MOTOR_PROFILE] is unchanged. */
    uint16_t sig = GSP_DefaultsSignature();
    memcpy(base + GSP_DEFAULTS_SIG_OFFSET, &sig, sizeof(sig));
#endif /* FEATURE_GSP_EEPROM */
}

#endif /* FEATURE_GSP */
