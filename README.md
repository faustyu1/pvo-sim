# ПВО — tactical simulator (Zig + raylib)

A 3D air-defence simulator with three ways to play:

- **Оборона — расчёт ПВО (defence, commander)**: you command a SAM battery (2 launchers, a radar and an AA gun) and defend a power plant, an oil depot and a residential block from waves of drones, decoys and cruise missiles.
- **Оборона — наводчик ЗСУ (defence, gunner)**: you sit in the AA gun and shoot drones down yourself through the sight.
- **Атака (attack)**: you fly a strike drone, and the same battery, now controlled by the AI, tries to shoot you down.

## Download

Ready-made builds for Windows, macOS (Apple Silicon and Intel) and Linux are on the [Releases](../../releases) page. Unzip and run `pvo` (`pvo.exe` on Windows). The `assets` folder must stay next to the executable.

macOS builds are not signed. If macOS refuses to open the game, run this once in the unzipped folder:

```bash
xattr -dr com.apple.quarantine pvo
```

Linux needs OpenGL 3.3 and X11 (or XWayland).

## Build from source

Requirements: Zig 0.16. raylib 6.2 is vendored in `vendor/raylib` and compiled from source, so nothing else needs to be installed on macOS or Windows. On Linux, install the X11 and GL headers:

```bash
sudo apt install libx11-dev libxrandr-dev libxinerama-dev libxi-dev libxcursor-dev libgl1-mesa-dev
```

Build and run:

```bash
zig build run
```

Cross-compile for Windows from any OS:

```bash
zig build -Dtarget=x86_64-windows-gnu
```

The default build is ReleaseFast. A Debug build (`-Doptimize=Debug`) is far too slow to play.

Models are generated procedurally in Blender (ambient occlusion is baked into vertex colours). To regenerate them into `assets/`:

```bash
/Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup --python tools/gen_models.py
```

## Controls

**Defence, commander view**
- RMB: rotate the camera. Wheel: zoom. WASD: pan.
- TAB or click: select a target. F or Space: launch a SAM.
- T: toggle auto-SAM. G: allow or forbid AA-gun fire.
- V: take the gunner seat.
- C: cycle camera (overview / missile / target). X: time acceleration. N: start the next wave now.

**Defence, gunner seat**
- Mouse: aim. LMB: fire.
- Wheel or Z: sight zoom.
- The laser rangefinder locks the target nearest the crosshair and sets the shell fuse. Put the crosshair on the small circle (the lead point), not on the target itself.
- Shells are simulated with air drag and gravity (1050 m/s at the muzzle, about 700 m/s at 2 km) and burst on a timed fuse. The lead point accounts for both. Every second round is a tracer.
- F: launch a SAM at the target in the sight. T: toggle auto-SAM, which is switched off when you take the seat.
- V: back to the commander view.

**Attack**
- Mouse or WASD: steer. Shift: boost.
- Wheel: camera distance. C: switch to nose camera. X: time acceleration.
- Below 60 m the radar only sees you up close, and hills block radar line of sight. When a missile is close, jink hard to make it miss.

Esc opens the pause menu, where you can save or go back to the main menu.

## Saves

Settings, records and the saved game are stored in `pvo_save.dat` next to the game.

## Settings

- Difficulty (easy / normal / hard). In defence it scales wave size, enemy speed and missile resupply. In attack it scales radar detection range, how long the AI needs to lock on, the pause between launches, missile and gun hit chance, and drone durability (10 / 6 / 3 hits).
- Rain, shadows, mouse sensitivity, Y inversion.
- Retina resolution, which takes effect after a restart. It is heavier on the GPU (about 40 FPS versus about 100 FPS at normal resolution).

## Third-party

- [raylib](https://github.com/raysan5/raylib) 6.2, zlib licence (`vendor/raylib/LICENSE`).
- PT Sans Narrow Bold font by ParaType, SIL Open Font Licence (`assets/FONT_LICENSE.txt`).
