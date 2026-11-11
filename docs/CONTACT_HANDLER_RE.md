# Car contact handler (static read, Steam exe PE timestamp 1789118616, 2026-11-11)

Function `0x142f04040` .. `0x142f04e22` (RVA 0x2f04040). It is a **contact modifier callback**, not the solver: it gets one contact
record in `rcx` (kept in `rsi`) and edits it before the engine's solver reads it. Full listing: ask for a re-dump (script in the
session: pefile + capstone, decode from 0x142f04040).

## Contact record (rsi)
| off | meaning | evidence |
|---|---|---|
| +0x32 | flag byte, bit 0 = **ignore this contact** (set by the handler itself when a normal comes out degenerate / on early-outs) | `or byte [rsi+32h],1` at 0x142f04496, 0x142f044a7, 0x142f049a5, 0x142f049c9 |
| +0x38 / +0x40 | the two collision objects; `[obj+0x150]->+0x18->+0x108->+0x458->+8` = the vehicle (veh) | 0x142f040fc.. |
| +0x48 / +0x50 | pointers to a 4-char type tag: 'CARW' (0x57524143, wheel), 'CARB' (0x42524143, body), 'PLYO', 'PLAS', 'CAR_' ... compared against object types; the tags are only checked when a global flag byte is set | 0x142f040bb.., 0x142f04215.. |
| +0x64 | **mass ratio** (float), written at 0x142f04871: `physics_A+0x3e4 / physics_B+0x3e4`, times `physics+0x3b4` for the car that is flagged human (`physics+0x350`) when exactly one of the two cars is human (the "player difficulty" factor, only on the car the handler chose as r14) | 0x142f04811..0x142f04871 |
| +0x70 | contact point (vec4) | wheel push reads it as xmm6 |
| +0x80 | contact normal (vec4, xyz) | assist A and the flattening code read and rewrite it |
| +0x84 | normal.y (read as a float): a branch at 0x142f04414 only runs when it is < 0 | 0x142f04414 |

## What the handler does (order)
1. Early branch on the two objects' type tags (CARW / CARB): decides if this is a car-car contact.
2. For a car-car contact picks the car `r14` that is "doing the hitting" from the sign of `dot(relative position, car forward)`
   (xmm5 vs the 0x1b0 forward vectors, uses `physics+0x1b0` / `+0x170` velocity) and stores a sideways-hit flag at `veh+0x8f0` (1 or 2).
3. Mass ratio `+0x64` as above (this is where player and AI differ: `+0x3e4` 50 vs 45, and `+0x3b4` scale for the human).
4. Assist B (DB bool, global 0x14606fc40-ish slot) and Assist A (slot used by our `softening` option, 0x142f04876): remove the
   component of the contact normal along the hitting car's **up** axis (`physics+0x1a0`) — the normal is projected into the car's
   ground plane, scaled by a DB value, renormalised, written back to `+0x80`, flag +0x32 set if the change is large. This is the
   code that makes wheel-over-wheel contacts push cars sideways and never upwards.
5. Early-out branch (0x142f04402..0x142f044a7) for other object types: if `contact+0x110 != 0` and `normal.y < 0` it flattens the
   normal to horizontal (y = 0) and normalises; flag +0x32 if the length changes.
6. Wheel-to-wheel push at the end (0x142f049f2, our `wheel_push` option): nearest wheel of each car to the contact point
   (`[[veh+0x580]+0x70+8i]`, position +0x2f0, velocity +0x3e0, +0x360 axis), then the game's apply-impulse function 0x1440c12e0
   on both cars (car[+0x130] bit 3 must be set).

## Why player and AI feel different (hypothesis from this read)
- Mass ratio gets the human difficulty factor only when exactly one car is human, and only for the car the handler picked.
- Assists A/B and the sideways flag are keyed on which car "does the hitting" (r14), so the result depends on car order and approach,
  not only on geometry: the same touch can be handled differently for car A/B.
- Our own AI-only extras (spin, twitch, lift) are never applied to a human car.

## Ideas that follow (not built)
- Replace the response for car-car contacts: apply a proper impulse (restitution, Coulomb friction, both inertias) with the game's
  apply-impulse function and set +0x32 bit 0 so the engine ignores its own response; needs positional correction for overlap.
- Lighter version: rewrite the contact normal (+0x80) from wheel geometry (keep the vertical part for tyre-on-tyre) and leave
  the engine's solver to do the rest.
- First step either way: log-only probe in the hook that records record fields (+0x32, +0x64, +0x70, +0x80, +0x84, object tags)
  and the engine's result per contact, for player-AI and AI-AI touches.
