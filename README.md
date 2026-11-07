# Contact Physics

Cars react to hits like real ones: the AI holds its steering for a moment and spins from the hit. Softening off, wheel-to-wheel push on. Experimental.

A Swofty Launcher addon for F1 25, by Kizikafuka. Version 0.5.9.

- Install it from the Swofty Addon Browser.
- Source: `native/src` (C++), manifest `swofty.json`. Shared headers are in `common/`.
- Build: Zig (`zig c++ -target x86_64-windows-gnu -std=c++23 -shared`) with the Swofty SDK headers in `native/include`.

License: MIT.
