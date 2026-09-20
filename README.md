# DigitalControl

DTU Digital Control – Basebot (Teensy 4.1) firmware and MATLAB analysis.

## Firmware

`basebot_6.ino`, built with PlatformIO (`platformio.ini`).

- `#define USE_SPIKE_SEQUENCE 0` → step sequence (1.5 / 3 / 6 V)
- `#define USE_SPIKE_SEQUENCE 1` → spike sequence (3 × 6 V, 10 ms)

Upload and monitor, lift the robot, type `start`, then `log`. The log is saved in `logs/`.

## MATLAB

Run from the repo root:

| Script | Does |
|---|---|
| `plotSteps.m` / `plotSpikes.m` | plot the step / spike logs |
| `motorParams.m` | R, L, Kb, J, D → `motorParams.mat` |
| `transferFunction.m` | robot mass, delay, Bode plot |
| `piDesign.m` | PI design + Simulink test (`motorPI.slx`) → `piDesign.mat` |

Results: [answers.md](answers.md)
