Contact Physics for F1 25 - Swofty Launcher addon (kizi.ai-tweaks) v0.8.2 - EXPERIMENTAL
==============================================================================================

From 0.6.0 this addon gives every car-to-car touch a rigid-body response: the push comes from both cars' mass and
inertia at the point where they touch, with bounce, friction and rolling tyres that rub against each other. AI cars are
pushed too (the game itself only pushes your car), and AI against AI touches count. The older contact (0.5: set spin, twitch,
tyre lift and wheel push) is back as a choice in 0.8.0: "Physics: contact model" Rigid body (default) or Classic.

WHO DOES WHAT: this addon = what happens to a CAR (contact, walls, aero, damage). AI Driver Behaviour
(kizi.ai-driver-behaviour) = how the AI DRIVES (racecraft, slides and spins, reaction time). No setting is in both.

This is an experiment. It changes how the game handles car contact in memory; the effect on racing is tuned by test
races, not proven, and may be unstable. Back up your save before using it.

WHAT IT CHANGES (memory only, nothing on disk)
  Contact physics: the game's car-to-car contact code edits each contact before the game solves it. The addon adds a
    small jump at the end of that code and, for every touch between two cars, works out an impulse from both cars'
    mass, inertia, speeds and the contact point (bounce, friction; tyre on tyre also with the tyres' rolling speed) and
    applies it with the game's own apply-impulse function. With "game's own response: Add" (default, 0.6.3) the game's
    solver does the main push of a hit, so the game's damage system sees how hard the hit was, and the addon adds the
    bounce, friction and tyre rub on top. At most 4 pushes per car pair per physics step (no build-up while paused or in a flashback).
  Walls: cars scrape along walls (friction) and barriers absorb most of a hit's energy (no pinball rebound).
  Energy-based wing damage: hard car-to-car hits add front-wing damage that grows with the energy of the hit.
  Aero loss when sideways: a sliding or spinning car loses most of its downforce (the game's own aero lift scale,
    through Swofty's live car field), so spins slide further.
  Softening: the two car-to-car contact assist switches (game: on) are set to off.
  AI driving is not in this addon (0.8.0): the AI's reaction after a hit and to a slide moved to AI Driver Behaviour
    (0.2.0 or newer). This addon finds the hits and passes them on; without AI Driver Behaviour there is no reaction hold.
  Punctures: a hard hit at a tyre can give it a puncture, through the game's own puncture function.
  Suspension after a jump: a car that flies and lands can get one corner of its suspension damaged (or broken), by
    chance; written into the game's own suspension damage counters, up to the game's own breaking point.
  Equal damage: AI cars get your car's damage factors (the game gives them different ones), so the same hit damages
    them as much as you.
  Terminal Damage: when its DLL is loaded, suspension damage is left to it (option, on by default).
  Contact log (testing only, off by default): writes touches to swofty\loader.log.

INSTALL
Needs Swofty Launcher 1.7.0 or newer. Drag this .zip onto the Swofty Launcher (Addons panel) and turn the addon on. Don't unzip it, don't run anything.
Settings: on the addon's row in the launcher, or ADDON SETTINGS in game. All settings change live.
Do not run it together with the test addons kizi.contact-sim-light / kizi.contact-sim-heavy (same hook).

WHAT IS IN THIS ZIP
  swofty.json               the addon's info for Swofty (name, version, settings, what to load)
  native\kizi_ai_tweaks.dll in-game part: contact physics, walls, aero loss, damage, punctures, jump damage
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
