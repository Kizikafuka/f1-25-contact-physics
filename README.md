# Contact Physics

What happens to a car in contact. Two contact models: Rigid body (the push comes from both cars' mass and inertia at the point where they touch, with real bounce and grip and rolling tyres that rub against each other) or Classic (the 0.5 contact). On top of that, "Crashes" (Realistic / Dramatic / Hollywood, P key in a race) sets how a crash plays out: a car thrown up at speed flies on a gravity curve, a spinning, broken or upside-down car slides and turns on, and a front tyre into a rear tyre can break that rear corner with a kick and a snap. Also wall physics, aero loss when sideways or nose up, punctures, loose wheels, jump damage and energy-based damage. AI cars get the same physics as you. How the AI drives (and reacts after a hit) is in AI Driver Behaviour 0.2.0 or newer. Its suspension damage has priority over Terminal Damage's. Experimental.

A Swofty Launcher addon for F1 25, by Kizikafuka. Version 1.2.2.

- Install it from the Swofty Addon Browser.
- Source: `native/src` (C++), manifest `swofty.json`. Shared headers are in `common/`: `kizi_contact_sim.hpp` is the contact physics, `kizi_mem.hpp` small memory helpers.
- Build: Zig (`zig c++ -target x86_64-windows-gnu -std=c++23 -shared`) with the Swofty SDK headers in `native/include`. See `release/BUILD.txt`.
- How the game's contact code works: `docs/CONTACT_HANDLER_RE.md`.
- Ideas for what comes next: `IDEAS.md`.

License: MIT.
