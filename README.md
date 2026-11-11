# Contact Physics

Rigid-body car contact for every car in F1 25: the push comes from both cars' mass and inertia at the point where they touch, with bounce, friction and rolling tyres that rub against each other. AI cars are pushed too, and the AI's reaction time after a hit grows with the size of the hit. Experimental.

A Swofty Launcher addon for F1 25, by Kizikafuka. Version 0.6.2.

- Install it from the Swofty Addon Browser.
- Source: `native/src` (C++), manifest `swofty.json`. Shared headers are in `common/`: `kizi_contact_sim.hpp` is the contact physics, `kizi_mem.hpp` small memory helpers.
- Build: Zig (`zig c++ -target x86_64-windows-gnu -std=c++23 -shared`) with the Swofty SDK headers in `native/include`. See `release/BUILD.txt`.
- How the game's contact code works: `docs/CONTACT_HANDLER_RE.md`.
- Ideas for what comes next: `IDEAS.md`.

License: MIT.
