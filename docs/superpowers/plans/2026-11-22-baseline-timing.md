# find_cup timing

Wall-clock seconds from the `[timing]` log lines. Sim: MuJoCo, osmesa, default scene.

## Before (old find_cup, 2026-11-22)

| Phase | Run 1 | Run 2 | Run 3 |
|---|---|---|---|
| AIM_HEAD | 11.6 | 11.5 | 11.5 |
| CENTER | 5.0 | 5.5 | 6.6 |
| READY | 19.5 | 19.3 | 19.4 |
| ALIGN | 42.1 (FAILED) | 64.0 (FAILED) | 42.2 (FAILED) |
| **Total** | **78.6, FAILED** | **102.8, FAILED** | **81.1, FAILED** |

All three failed in ALIGN with "lost the cup in depth during alignment". The cup started
about +11.7 cm from the arm line, and the base barely moved forward: one nudge at 0.04 m/s
for 6 s moved it 0.2 cm, while CENTER had driven it normally a few seconds earlier.
