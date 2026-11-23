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

## After (new find_cup, `-p grasp:=true`, 2026-11-22, Mac VM, osmesa, ~1% real time)

Sim-time seconds (what the real robot would roughly take); wall time in brackets.

| Phase | Run B3 |
|---|---|
| AIM_HEAD | 0.4 (28.9 wall) |
| READY | 2.4 (317 wall) |
| ALIGN | 8.5 (1006 wall): one 12.2 cm drive + wrist yaw, grasp point -0.1 cm from the cup |
| REACH | 4.4 (523 wall) |
| **Total** | **15.7, DONE** (1875 wall) |

The old runs' wall times are not comparable: the old code timed with the wall clock and
moved on before the sim finished each move (the base barely moved in ALIGN).

## Verification runs on the desktop (2026-11-22, WSL2, patched sim from PR #6)

MuJoCo viewer on, default scene, fresh sim per run. With the patches the sim runs at real
time (GPU, `MUJOCO_GL=egl`), so sim and wall time match.

| Run | Arguments | Result |
|---|---|---|
| A | `grasp:=true use_wrist_yaw:=false` | DONE in 20.8 s: one 12.8 cm drive, cup -0.6 cm from the arm line; lift 0.415 / extension 0.139 for targets 0.418 / 0.141 |
| D1 | `odom_topic:=/nope` | FAILED naming `/nope`, but only after the 30 s startup timeout and with a camera hint; now fails after 5 s with an odometry hint |
| D2 | `head_pan:=0.0` | FAILED after 4.1 s: "no red blob in the image (0 red pixels)"; all 43 `/stretch/cmd_vel` messages were 0.0; odometry unchanged afterwards |
| C | `nudge_base.py 0.09`, then `grasp:=true` | wrist turned +0.121 rad, grasp point +0.1 cm from the cylinder, but the extension was blocked 2.8 cm short and the run still said DONE; now FAILED "the arm was blocked before reaching the cup" |

Run C's blocked reach was likely a finger hitting the 500 g cylinder or the box 10 cm
next to it in the default scene; with the realistic task-scene cups the wrist-yaw path
reached and lifted (roadmap 1.6 to look into it).
