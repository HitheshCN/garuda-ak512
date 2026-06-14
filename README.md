# garuda-ak512

Garuda sensorless motor-control firmware for the **dsPIC33AK512MC510** Motor
Control DIM (EV67N21A) on the Microchip **MCLV-48V-300W** development board.

A port of the Garuda 6-step + AN1078 FOC firmware (originally
dsPIC33AK128MC106) to the dsPIC33AK512MC510. The Garuda HAL is the isolation
boundary, so the control core is shared — this port is primarily a peripheral /
pin remap. See `docs/migration_dsPIC33AK512MC510.md` for the plan and
`docs/port-ak512/` for bring-up notes.

## Build

| Setting | Value |
|---|---|
| IDE / project | MPLAB X — `dspic33AK512MC510.X` |
| Device | dsPIC33AK512MC510 |
| Toolchain | XC-DSC 3.30 |
| DFP | dsPIC33AK-MC_DFP 1.4.172 |
| Programmer | on-board PKoB4 (`pkob4hybrid`) |

## Layout

- `dspic33AK512MC510.X/` — MPLAB X project
- `hal/` — board/peripheral abstraction (the port boundary)
- `motor/`, `gsp/`, `scope/`, `input/`, `learn/`, `x2cscope/` — firmware modules
- `foc/` — AN1078 SMO FOC + FOC v2/v3
- `main.c`, `garuda_*.{c,h}` — control core
- `docs/` — migration plan, bring-up + research notes
- `releases/` — validated build artifacts

## Status

HAL port + bring-up complete; closed-loop idle solid (0% capture miss on
day-1 bench). Active development.

## Companion

Bench GUI / live tuning: **garuda-studio** (web) — connects over the GSP serial
protocol via Web Serial.
