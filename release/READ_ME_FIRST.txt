Contact Physics for F1 25 - Swofty Launcher addon (kizi.ai-tweaks) v0.6.2 - EXPERIMENTAL
==============================================================================================

From 0.6.0 this addon gives every car-to-car touch a rigid-body response: the push comes from both cars' mass and
inertia at the point where they touch, with bounce, friction and rolling tyres that rub against each other. AI cars are
pushed too (the game itself only pushes your car), and AI against AI touches count. The older made-up spin, twitch, lift
and wheel-push settings are gone; the physics does that work now. Racecraft and car control settings are in the addon
AI Driver Behaviour (kizi.ai-driver-behaviour).

This is an experiment. It changes how the game handles car contact in memory; the effect on racing is tuned by test
races, not proven, and may be unstable. Back up your save before using it.

WHAT IT CHANGES (memory only, nothing on disk)
  Contact physics: the game's car-to-car contact code edits each contact before the game solves it. The addon adds a
    small jump at the end of that code and, for every touch between two cars, works out an impulse from both cars'
    mass, inertia, speeds and the contact point (bounce, friction; tyre on tyre also with the tyres' rolling speed) and
    applies it with the game's own apply-impulse function. The game still keeps the cars from overlapping ("game's own
    response: Add"). At most 4 pushes per car pair per physics step (no build-up while paused or in a flashback).
  Softening: the two car-to-car contact assist switches (game: on) are set to off.
  AI reaction time: after a hit the AI car's steering is held at its value from the moment of the hit, so the hit can
    turn the car: 0.1 s for a light nudge up to 1.0 s for a big hit (or a fixed 0.2 / 0.4 / 0.6 s).
  Punctures: a hard hit at a tyre can give it a puncture, through the game's own puncture function.
  Contact log (testing only, off by default): writes touches to swofty\loader.log.

INSTALL
Needs Swofty Launcher 1.7.0 or newer. Drag this .zip onto the Swofty Launcher (Addons panel) and turn the addon on. Don't unzip it, don't run anything.
Settings: on the addon's row in the launcher, or ADDON SETTINGS in game. All settings change live.
Do not run it together with the test addons kizi.contact-sim-light / kizi.contact-sim-heavy (same hook).

IN A RACE (game window focused)
  Ctrl+F9    contact softening on / off (high beep = off = realistic, low beep = game default)
Hotkey changes last until the game closes; set the value in the settings to keep it.

WHAT IS IN THIS ZIP
  swofty.json               the addon's info for Swofty (name, version, settings, what to load)
  native\kizi_ai_tweaks.dll in-game part: contact physics, softening, reaction time, punctures, hotkey, messages
  icon.png                  the addon's logo
  source_code\              the complete source code and how to build it
  LICENSE.txt, THIRD_PARTY_LICENSES.txt

WHAT IT DOES AND DOESN'T DO
- No internet, installs nothing, nothing at startup, no registry or system changes, starts no other programs.
- The DLL only touches the memory of the game it runs in (F1 25). It writes no files.
- Game files on disk are not changed.

WHY ANTIVIRUS MIGHT COMPLAIN
The DLL is a small native file that is not code-signed. The source code is included so anyone can check it.

CREDITS
Swofty for the Swofty Launcher and its MIT-licensed SDK. Made with the help of an AI assistant.
