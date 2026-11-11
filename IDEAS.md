# Ideas for the contact / physics revamp

Realism first. Written after 0.6.2 (2026-11-11). Nothing here is built yet. "Known" means we already found the game code or data
it needs; "research" means that part is still unknown.

## Suggested order
1. Speed-dependent bounce (#1) and sliding-speed friction (#3): quick, in our own code, every touch gets more believable.
2. Remove the player's collision assistance (#15): one database edit, makes your car and the AI equal.
3. Spin probe (#13), then aero loss when sideways (#11) and AI braking in spins (#12): long slides in high-speed crashes.
4. Damage from impact energy (#6): the biggest unknown, and it unlocks #7, #8, #10 and #14.

Later (noted 2026-11-11): an optional "extra AI spin" on top of the physics, only if tests show AI cars rarely spin after hits
that should spin them. The old Classic spin / twitch / rear-hit code is still in `native/src/ai_tweaks.cpp`.

## A. Make the contact itself more accurate

| # | Idea | Why it is more realistic | What is needed |
|---|---|---|---|
| 1 | Speed-dependent bounce | A low-speed tap bounces a little, a big hit crumples and absorbs the energy. Today bounce is one fixed value (0.25). | Easy: bounce as a curve over the closing speed. |
| 2 | Tread vs sidewall | Tread on tread grips, sidewall on sidewall slides: side-by-side wheel touches should glance, a front tyre into a rear tyre should grab. | Easy-medium: wheel position known, wheel axis probably wheel +0x360. |
| 3 | Rubber friction drops at high sliding speed | Rubber grips less the faster it slides; very fast wheel rubs (40 m/s surface speed) should grip less than slow ones. | Easy: friction as a curve over the sliding speed. |
| 4 | Car-to-wall contacts | Only car-car touches use the new physics; walls still use the game's "pinball" response. A real car scrapes along a wall. | Medium: the hook already sees 4,000-20,000 wall records per 30 s (skipped today); needs the barrier type and infinite mass. |
| 5 | Which part of the car touched | Nose into sidepod differs from wing on wing. The game tags body / wheel; there may be finer parts. | Research: more of the contact record (type tags, other objects). |

## B. Consequences of a hit

| # | Idea | Why | What is needed |
|---|---|---|---|
| 6 | Damage from impact energy | Wing / floor / sidepod damage in proportion to the energy of the hit; we already compute the impulse of every touch. | Research: where the game stores per-car damage in memory (the telemetry packet is known, the memory location is not). |
| 7 | Front wing failure from tyre contact | The classic lap-1 incident: an endplate clips a rear tyre and the wing breaks. | Needs #6 plus "front-wing point near a rear wheel". |
| 8 | Suspension damage that changes handling | After a hard wheel hit the car pulls to one side or wobbles. | Research: the suspension object (one per car; car link not found yet). |
| 9 | Punctures v2 | A sidewall cut from a front-wing endplate, or a slow puncture after wheel contact; confirm the tyre visibly deflates. | Medium: the puncture function works; check deflation, add the "cut" case. |
| 10 | Retire after a huge crash | A 20 m/s+ hit into a car or a wall sometimes ends the race. | Known: DNF / terminal-damage paths (canned DNFs, terminal-damage research). |

## C. After the hit: slides, spins, crashes

| # | Idea | Why | What is needed |
|---|---|---|---|
| 11 | Aero loss when sideways or backwards | Why spinning F1 cars slide so far: the wings stall and downforce is gone. In the game a spun car stops after about 1.1 s whatever its speed. | Research: a per-car grip / downforce lever (tyre types per car known, GripMultiplier +0x2c; car-to-tyre-type link not). Probe first. |
| 12 | The AI does not slam the brakes in a spin | The AI brakes 100 % within 0.1-0.9 s of breaking away, which kills the slide. | Database edits ("Spinning out" / "Lost control" blocks) in AI Driver Behaviour; earlier tests inconclusive. |
| 13 | Spin / slide probe | Log every spin: speed at breakaway, distance, time, when the AI brakes. Data before tuning #11 / #12. | Easy: log only, same body fields as now. |
| 14 | No grip when launched, hard landings | A car in the air or on two wheels has no grip; a launch should end with a hard landing and possible damage. | Needs #6 and landing detection (vertical speed). |

## D. Fairness and feel: you against the AI

| # | Idea | Why | What is needed |
|---|---|---|---|
| 15 | Remove the player's collision assistance | The game makes your car heavier in contacts (collision mass 50 vs 45) and scales it with difficulty: part of why your car "corrects itself". | Known: `collision_assistance.collisionMassModifier_human/_ai`, `difficulty.collision_impact_modifiers`; set human = AI by database edit. |
| 16 | Force-feedback jolt on contact | Feel the hit in the wheel, scaled by our impulse; a side touch kicks the steering. | Medium: FFB link research exists; the impulse is the input. |
| 17 | Reaction time per driver | Experienced / high-rated drivers react faster after a hit than rookies. | Medium: driver ratings link (participants -> driver, Driver Probe research). |

## E. Race-level realism

| # | Idea | Why | What is needed |
|---|---|---|---|
| 18 | Debris after a hit | Wing pieces on track, puncture risk for following cars; the game has debris-puncture keys. | Research: the game's debris system. |
| 19 | Safety car / VSC after a big crash | A car stopped or wrecked after a big hit brings out the safety car. | Known: the canned addon's safety-car trigger; needs "stopped after a big impact" detection. |
| 20 | Contact report | An incident file per race: who hit whom, how hard, where, the result. A tuning tool with real data, and fun to read. | Easy: everything per touch is already logged. |
