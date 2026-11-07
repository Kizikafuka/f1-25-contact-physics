Contact Physics for F1 25 - Swofty Launcher addon (kizi.ai-tweaks) v0.5.0 - EXPERIMENTAL
==============================================================================================

From 0.5.0 this addon only does what a hit does. The racecraft and car control settings moved to the addon
AI Driver Behaviour (kizi.ai-driver-behaviour).

This is an experiment. It changes game values that the game does not normally expose; the effect on racing is not
proven and may be unstable. Back up your save before using it.

WHAT IT CHANGES
  Live (written into the game's memory by the DLL, the game reads them on every car-to-car contact):
    the two car-to-car contact assist switches (game: on)
    the wheel-to-wheel contact values grip / push scale / push cap (game: -1 / 0 / 0, i.e. off)
  Equal weight (changes one instruction of the game's contact code in memory, nothing on disk):
    right after the game works out a contact's mass ratio (your car counts heavier than an AI car), the ratio is set
    to 1.0 while the option is on. The same spot records each contact for the two features below.
  AI reaction time (memory only): every frame the AI hands its steering to its car; after a hit that AI car's
    steering is held at its value from the moment of the hit for 0.2 / 0.4 / 0.6 s, like a driver who has not
    reacted yet, so the hit can turn the car.
  AI spin from hits (memory only): on a new contact the AI car gets the spin the hit gives (from the contact point,
    the direction of the push and the closing speed), held for a quarter of a second.
  Contact log (testing only, off by default): writes every car-to-car touch to swofty\loader.log.

INSTALL
Needs Swofty Launcher 1.7.0 or newer. Drag this .zip onto the Swofty Launcher (Addons panel) and turn the addon on. Don't unzip it, don't run anything.
Settings: on the addon's row in the launcher, or ADDON SETTINGS in game. All settings change live.

IN A RACE (game window focused)
  Ctrl+F9    contact softening on / off (high beep = off = realistic, low beep = game default)
  Ctrl+F10   wheel-to-wheel push: off, subtle, light, medium, strong, extreme (beeps = level; long low beep = off)
Hotkey changes last until the game closes; set the value in the settings to keep it.

WHAT IS IN THIS ZIP
  swofty.json               the addon's info for Swofty (name, version, settings, what to load)
  native\kizi_ai_tweaks.dll in-game part: the live values and contact features above, hotkeys, messages
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
