# fw2-lander

A lunar landing game for the FreeWili 2 DISPLAY CPU, flown on a DSKY-style
screen. The descent model and its guidance follow the Apollo 11 landing
software (Luminary 099, `LUNAR_LANDING_GUIDANCE_EQUATIONS.agc`):

- **P64 (approach):** quadratic guidance. Time-to-go is advanced each pass
  and refined by Newton iteration on the downrange cubic
  `JDG·T³ + 6·ADG·T² + (6·VG + 18·VDG)·T + 24·(RDG − RG) = 0`, then the
  commanded acceleration is `ACG = ADG + 6(VDG + VG)/TTF + 12(RDG − RG)/TTF²`.
- **P66 (rate of descent):** below 150 ft (or when time-to-go runs out),
  `ACG = (VDGVERT − VG)/TAUVERT` with a descent rate that shrinks with
  altitude.
- **Program alarm 1202** fires once at 2000 ft, as it did on Apollo 11: the
  DSKY flashes V05 N09 with `01202`, guidance drops out briefly while the
  computer restarts, and the PROG and RESTART lamps latch until RSET.

You fly; guidance shows what it would command (magenta marks on the GUID
panel: throttle tick and attitude diamond).

## Controls

| Input | Action |
| --- | --- |
| Tilt the board left/right | LM pitch (BMI323 accelerometer) |
| Hold GREEN | Throttle up (release = DPS minimum, 10%) |
| RED | RSET: clear PROG/RESTART, dismiss V05 N09 |
| NAV LEFT/RIGHT | Pitch trim (the only steering if no IMU is found); NAV CENTER zeroes it |
| Hold PAGE 5 s | About screen |
| Hold HOME 5 s | Leave the app |

Before the first flight the app learns how you hold the board: hold it the
way you will fly and press GREEN, then tilt it right and press GREEN.

## Screen

- R1 altitude (ft), R2 altitude rate (0.1 ft/s), R3 fuel (%), V06 N60.
- Lamps: PROG/RESTART (1202), ALT (descending too fast for the altitude),
  VEL (horizontal speed too high near the ground), GIMBAL LOCK (pitch over 40°).
- Messages: `60 SECONDS` / `30 SECONDS` fuel callouts, `CONTACT LIGHT`.
- Soft landing: descent ≤ 2 m/s (6.6 ft/s), horizontal ≤ 1.2 m/s, pitch ≤ 12°.
- LEDs: throttle level, red flashing during the alarm, green/red at the end.

## Getting started

```
git clone --recurse-submodules https://github.com/rquinn-intrepid/fw2-lander.git
cd fw2-lander
```

Needs the Pico SDK 2.3.0 toolchain under `~/.pico-sdk` (the Raspberry Pi Pico
VS Code extension installs it), Python 3, CMake and Ninja; see
`wilibsp/README.md`. For `tools\dev test` on Windows, MSYS2's mingw64 GCC.

| Command | What it does |
| --- | --- |
| `tools\dev build` | Build `build/lander.uf2` |
| `tools\dev run` | Build, load into the board over USB and start it (gone after a reboot) |
| `tools\dev install` | Copy the UF2 to the SD card's `/apps/` so it shows in the board's app list |
| `tools\dev test` | Host unit tests: the guidance flies the model to a soft landing |
| `tools\dev screenshot -o shot.png` | Any other wilibsp `fw` verb works the same way |

`wilibsp/` is a submodule pinned to the BSP commit this app was tested
against.