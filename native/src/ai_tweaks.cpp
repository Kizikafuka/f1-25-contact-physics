// SPDX-License-Identifier: MIT
// AI Racing Tweaks, live part. Works on any Swofty install: no hooks and no extra bindings, only Swofty's pattern
// search (api->scan) to find the game's own contact settings, which the game reads on every car-to-car contact.
//
// 1. Contact softening. The car-to-car contact handler runs two "assists" that soften every hit. Each is switched
//    by a game-database bool the handler reads through a pointer slot:
//      mov rax,[rip+slot]; mov rax,[rax]; test rax,rax; je skip; cmp byte [rax+8],0; je skip   (and the same with rcx)
//    Setting that bool to false switches the assist off for every contact.
// 2. Wheel-to-wheel push. At the end of the handler, touching tyres grip each other and push both cars at the
//    contact point: min(relative wheel speed x scale, cap), only when grip > 0. The game database ships it off
//    (grip -1, scale 0, cap 0). The three floats sit together; the handler reads them with
//      vmovss xmm0,[rip+params]; vxorps xmm1,xmm1,xmm1; vucomiss xmm1,xmm0; jae skip
// The game copies these values from its database when the collision system starts, possibly after this plugin
// loads, so a worker re-applies the chosen state about once a second.
//
// No hotkeys since 0.6.6 (Ctrl+F9 softening / Ctrl+F10 wheel push removed); everything is set in the addon settings.
// a message also shows in the game's race notification.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <vector>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <string_view>
#include <thread>

#include <swofty/overlay.hpp>
#include "osd.hpp"
#include "kizi_mem.hpp"
#include <swofty/events.hpp>
#include <swofty/systems/cars.hpp>
#include "osd.hpp"

SWOFTY_SETTING(bool, contact_log, false);
SWOFTY_SETTING(std::string_view, puncture, "sometimes");
// Classic (0.9.11): one "Classic: strength" preset instead of six settings. Each old setting is now a pick from the preset
// (Light / Medium / Strong / Wild); Strong = the old defaults.
SWOFTY_SETTING(std::string_view, classic_strength, "strong");
struct ClassicPick {
    std::string_view light, medium, strong, wild;
    std::string_view get() const
    {
        std::string_view v = classic_strength.get();
        return v == "light" ? light : v == "medium" ? medium : v == "wild" ? wild : strong;
    }
};
const ClassicPick contact_spin{"light", "medium", "strong", "strong"};
const ClassicPick twitch{"off", "small", "medium", "medium"};
const ClassicPick spin_target{"both", "both", "both", "both"};
const ClassicPick rear_boost{"small", "medium", "strong", "strong"};
const ClassicPick wheel_lift{"small", "medium", "strong", "strong"};
const ClassicPick wheel_push{"subtle", "light", "medium", "extreme"};
// 1.0.0: fewer settings. Merged: "your car too" (was punctures + suspension damage for your car), "suspension after a jump"
// Off / Damage / Break (was chance + level; the chance is now Sometimes). Fixed: contact applies to all touches, the game's
// contact softening off, wall physics on, suspension left to Terminal Damage when its suspension model runs. The old names
// stay as read-only stand-ins so the code below reads as before.
SWOFTY_SETTING(bool, your_car, true);
SWOFTY_SETTING(std::string_view, jump, "damage");
struct Fixed {
    std::string_view v;
    std::string_view get() const { return v; }
};
struct FixedBool {
    bool v;
    bool get() const { return v; }
    operator bool() const { return v; }
};
struct YourCar {
    bool get() const { return your_car.get(); }
    operator bool() const { return your_car.get(); }
};
struct JumpPick {
    bool level;                                    // false: the chance, true: the level
    std::string_view get() const
    {
        std::string_view j = jump.get();
        return level ? (j == "break" ? "break" : "damage") : (j == "off" ? "off" : "sometimes");
    }
};
const Fixed spin_who{"all"}, softening{"off"};
const YourCar puncture_you, jump_you;
const JumpPick jump_break{false}, jump_damage{true};
SWOFTY_SETTING(std::string_view, rear_break, "sometimes");
SWOFTY_SETTING(std::string_view, loose_wheels, "all");
SWOFTY_SETTING(bool, equal_damage, true);
SWOFTY_SETTING(std::string_view, front_tough, "tougher");
// 1.2.2: Contact Physics has priority over Terminal Damage's suspension model (user decision): td_defer off = all of this
// addon's suspension damage runs whatever Terminal Damage's options are; a small notice says so (notice_tick).
const FixedBool td_defer{false}, walls{true};
SWOFTY_SETTING(std::string_view, damage_boost, "normal");
// Physics mode ("Physics: contact model"). "realistic" = Rigid body: the response of common/kizi_contact_sim.hpp (impulse from
// both cars' mass and inertia at the contact point, bounce, friction, rolling tyres rubbing, AI cars pushed too); the only
// model in 0.6.1 - 0.7.3. "classic" = the 0.5.x contact (the spin / twitch / rear-hit / lift / wheel push below), back as a
// choice in 0.8.0 because players missed the flying on tyre contact. Walls, aero loss and damage work with both.
SWOFTY_SETTING(std::string_view, physics_mode, "realistic");
// 1.1.0 overhaul: "Crashes" replaces the six Rigid body presets (1.0 presets scaled every touch: taps bounced and grabbed,
// "stiff", while big crashes still looked tame). Touches now use real values for every level (see kizi_contact_sim.hpp);
// the level only changes what happens in a crash:
//              big-hit  wheel-over-wheel  flight: pop up to /  nose-up downforce  sideways   rear break: snap  kick / launch
//              bounce   lift x / cap m/s  gravity (see Fly)    (negative = lift)  downforce  yaw (170 km/h)    (m/s up)
//   realistic  x1       x1   / 5          5 m/s / 9.8 m/s2     0                  30 %       70 deg/s          - / -
//   dramatic   x1.4     x1.8 / 7          6 m/s / 8.5 m/s2     0                  12 %       150 deg/s         1.8 / 2.7
//   hollywood  x2.2     x3   / 10         8 m/s / 6.5 m/s2     -0.3               5 %        260 deg/s         3.0 / 4.5
// 1.1.1: flights. A car thrown up at speed now flies on a gravity curve (it was pressed back down by its downforce within
// 0.1 s), so the lift caps went up; lift below 0 (unverified in the game) only on Hollywood, the curve does the rest.
// 1.2.0, slides (see Slide): of the speed / yaw rate a sliding or spinning car loses, this share is given back:
//   realistic 40 / 35 %, dramatic 50 / 55 %, hollywood 65 / 70 % (upside down: 75 % of the speed at every level).
// Read on every touch / tick, so a change (setting or P key) applies at once, even in a race.
SWOFTY_SETTING(std::string_view, crashes, "dramatic");
struct Crash {
    float big_bounce, launch_boost, launch_cap, lift_min, yaw_floor, snap_dps, kick_up, fly_cap, fly_g, slide_keep, spin_keep;
};
constexpr std::string_view CRASH_LEVELS[3] = {"realistic", "dramatic", "hollywood"};
std::atomic<int> g_level_key{-1};                  // the P key's choice (until the game closes); -1 = the setting
std::string_view level_name()
{
    int k = g_level_key.load();
    return k >= 0 && k < 3 ? CRASH_LEVELS[k] : crashes.get();
}
Crash crash()
{
    std::string_view v = level_name();
    return v == "realistic"   ? Crash{1.0f, 1.0f, 5.0f, 0.0f, 0.3f, 70.0f, 0.0f, 5.0f, 9.81f, 0.4f, 0.35f}
           : v == "hollywood" ? Crash{2.2f, 3.0f, 10.0f, -0.3f, 0.05f, 260.0f, 3.0f, 8.0f, 6.5f, 0.65f, 0.7f}
                              : Crash{1.4f, 1.8f, 7.0f, 0.0f, 0.12f, 150.0f, 1.8f, 6.0f, 8.5f, 0.5f, 0.55f};
}
// Per-level numbers (1.0.1), to tell the levels apart from a log: every touch counts for the level that was on (0-2, 3 =
// Classic). Printed for a level when P leaves it, and every 5 minutes for all that had touches.
struct PresetStats {
    std::atomic<int> touches{0}, breaks{0}, launches{0}, kicks{0}, logged{0}, snaps{0}, blowovers{0}, flights{0}, slides{0};
    std::atomic<float> closing_sum{0}, closing_max{0}, side_sum{0}, spin_max{0}, aoa_max{0}, fly_max{0}, slide_max{0};
};
PresetStats g_pstats[4];
int preset_index()
{
    if (physics_mode.get() != "realistic")
        return 3;
    for (int k = 0; k < 3; k++)
        if (CRASH_LEVELS[k] == level_name())
            return k;
    return 1;
}
std::string_view index_name(int k) { return k < 3 ? CRASH_LEVELS[k] : "classic"; }
void add_max(std::atomic<float> &a, float v)
{
    for (float o = a.load(); v > o && !a.compare_exchange_weak(o, v);)
        ;
}
void add_sum(std::atomic<float> &a, float v)
{
    for (float o = a.load(); !a.compare_exchange_weak(o, o + v);)
        ;
}

void sim_lifted(uint64_t body, float dv);           // flights (end of this file)
namespace sim::cfg {
inline bool active() { return physics_mode.get() == "realistic"; }
inline std::string_view normal_mode() { return "full"; }
inline std::string_view apply_to() { return spin_who.get(); }
inline bool contact_log() { return ::contact_log.get(); }
inline std::string_view restitution() { return "medium"; }   // 1.1.0: walls keep half of 0.25 (car-car: own real values)
inline std::string_view friction() { return "medium"; }
inline std::string_view separation() { return "low"; }
inline std::string_view engine_response() { return "add"; }
inline std::string_view exaggerate() { return "off"; }        // walls: x1
inline float big_bounce() { return crash().big_bounce; }
inline float launch_boost() { return crash().launch_boost; }
inline float launch_cap() { return crash().launch_cap; }
inline bool tyre_spin() { return true; }
inline bool ai_impulse() { return true; }
inline bool walls() { return ::walls.get(); }
inline float damage_boost()
{
    std::string_view v = ::damage_boost.get();
    return v == "normal" ? 1.0f : v == "high" ? 2.0f : 0.0f;
}
inline std::string_view safety_limits() { return "all"; }     // soft touches; the launch cap comes from the level
inline void lifted(uint64_t body, float dv) { sim_lifted(body, dv); }
}  // namespace sim::cfg
#define KIZI_SIM_EMBED 1
#define KIZI_SIM_HEAVY 1
#define SIM_TAG "[ai-tweaks] realistic:"
#include "kizi_contact_sim.hpp"

namespace {

const char *const ASSIST_B = "48 8B 05 ?? ?? ?? ?? 48 8B 00 48 85 C0 0F 84 ?? ?? ?? ?? 80 78 08 00 0F 84 ?? ?? ?? ?? "
                             "49 8B 86 58 05 00 00";
const char *const ASSIST_A = "48 8B 0D ?? ?? ?? ?? 48 8B 09 48 85 C9 0F 84 ?? ?? ?? ?? 80 79 08 00 0F 84 ?? ?? ?? ?? "
                             "48 8B 8F 58 05 00 00";
const char *const WHEEL = "C5 FA 10 05 ?? ?? ?? ?? C5 F0 57 C9 C5 F8 2E C8 0F 83";

uintptr_t g_slot[2];                               // pointer slots of the two assist bools
uintptr_t g_params;                                // wheel-to-wheel grip, scale, cap

struct Push {
    const char *name;
    float grip, scale, cap;
};
// Levels from test races: pushes of 1500+ sent cars flying.
const Push PUSH[] = {{"game (off)", -1, 0, 0},     {"subtle", 1.0f, 25, 100},  {"light", 1.0f, 50, 250},
                     {"medium", 1.0f, 100, 500},  {"strong", 1.0f, 200, 1000}, {"extreme", 1.0f, 400, 1500}};
constexpr int LEVELS = sizeof PUSH / sizeof PUSH[0];

std::atomic<int> g_push_key{-1};                   // -1 = from settings; else a PUSH index chosen with Ctrl+F10
std::atomic<int> g_soft_key{-1};                   // -1 = from settings; 0 = game softening, 1 = off (Ctrl+F9)
bool g_wrote_push, g_wrote_soft;                   // only put the game's values back after we changed them

// 3. Equal weight. Right before assist A the handler writes the contact's mass ratio (contact+0x64 = car B's
//    collision mass factor / car A's; the player's factor is scaled with difficulty), so one car acts much heavier
//    and the AI barely moves on contact. The `mov rcx,[rip+slot]` that starts the ASSIST_A pattern (rsi = contact)
//    jumps to a stub that runs our in-step hook, then does that mov and jumps back. (The equal-weight option that used to
//    set the ratio to 1.0 here was removed in 0.5.5: it made the physics feel wrong. The flag byte stays 0.)
uint8_t *g_eq_flag;                                // 1 = equal weight on (read by the stub)
uint32_t *g_eq_count;                              // contacts made equal weight (the stub counts them)

// 4. Contact log (testing). The stub also records every contact the handler sees, before the ratio is changed:
//    {contact, car A (rdi), car B (rbx), the game's ratio}, in a ring the worker reads. A car's physics object is
//    vehicle+0x558: +0x350 human flag (the handler's own test), +0x170 velocity, +0x3e4 collision mass factor.
struct Rec {
    uint64_t contact, a, b;
    float ratio;
    uint32_t pad;
    float point[4], normal[4];                     // contact+0x70 / +0x80 at the time of the record
};
static_assert(sizeof(Rec) == 64);
constexpr uint32_t RING = 2048;                    // entries (power of two)
uint8_t *g_log_flag;                               // 1 = record (read by the stub)
uint32_t *g_log_next;                              // entries written so far (the stub counts)
Rec *g_ring;

void on_record(uint64_t contact, uint64_t a, uint64_t b);   // in-step hook (below)

bool install_equal_weight(uintptr_t hit, uintptr_t slot)
{
    uint8_t first[3] = {};
    if (!hit || !slot || !kizi::read(hit, first, 3) || first[0] != 0x48 || first[1] != 0x8B || first[2] != 0x0D)
        return false;
    uint8_t *m = kizi::alloc_near(hit, 4096);
    if (!m)
        return false;
    uint8_t *ring = kizi::alloc_near(hit, RING * sizeof(Rec));
    if (!ring)
        return false;
    uint8_t *flag = m + 2048;                          // data after the code
    auto *count = reinterpret_cast<uint32_t *>(m + 2052);
    uint8_t *lflag = m + 2056;
    auto *lnext = reinterpret_cast<uint32_t *>(m + 2060);
    uint8_t *p = m;
    auto rel = [](uint8_t *from_next, void *to) { return int32_t(static_cast<uint8_t *>(to) - from_next); };
    auto put32 = [&](int32_t v) { memcpy(p, &v, 4), p += 4; };
    auto put64 = [&](uint64_t v) { memcpy(p, &v, 8), p += 8; };
    // contact log: if (lflag) { i = lnext++ (atomic); ring[i % RING] = {rsi, rdi, rbx, [rsi+64h]}; }
    *p++ = 0x80, *p++ = 0x3D;                          // cmp byte [rip+lflag],0
    put32(rel(p + 5, lflag));
    *p++ = 0x00;
    uint8_t *skip = p + 1;
    *p++ = 0x74, *p++ = 0x00;                          // je skip (patched below)
    *p++ = 0x50, *p++ = 0x52;                          // push rax; push rdx
    *p++ = 0xB8, put32(1);                             // mov eax,1
    *p++ = 0xF0, *p++ = 0x0F, *p++ = 0xC1, *p++ = 0x05; // lock xadd [rip+lnext],eax
    put32(rel(p + 4, lnext));
    *p++ = 0x25, put32(int32_t(RING - 1));             // and eax,RING-1
    *p++ = 0xC1, *p++ = 0xE0, *p++ = 0x06;             // shl eax,6 (64-byte entries)
    *p++ = 0x48, *p++ = 0x8D, *p++ = 0x15;             // lea rdx,[rip+ring]
    put32(rel(p + 4, ring));
    *p++ = 0x48, *p++ = 0x01, *p++ = 0xC2;             // add rdx,rax
    *p++ = 0x48, *p++ = 0x89, *p++ = 0x32;             // mov [rdx],rsi
    *p++ = 0x48, *p++ = 0x89, *p++ = 0x7A, *p++ = 0x08; // mov [rdx+8],rdi
    *p++ = 0x48, *p++ = 0x89, *p++ = 0x5A, *p++ = 0x10; // mov [rdx+16],rbx
    *p++ = 0x8B, *p++ = 0x46, *p++ = 0x64;             // mov eax,[rsi+64h]
    *p++ = 0x89, *p++ = 0x42, *p++ = 0x18;             // mov [rdx+24],eax
    for (uint8_t o : {0x70, 0x78, 0x80, 0x88}) {       // contact point and normal: mov rax,[rsi+o]; mov [rdx+o-50h],rax
        *p++ = 0x48, *p++ = 0x8B, *p++ = 0x86, put32(o);
        *p++ = 0x48, *p++ = 0x89, *p++ = 0x42, *p++ = uint8_t(o - 0x50);
    }
    *p++ = 0x5A, *p++ = 0x58;                          // pop rdx; pop rax
    *skip = uint8_t(p - (skip + 1));
    *p++ = 0x80, *p++ = 0x3D;                          // cmp byte [rip+flag],0
    put32(rel(p + 5, flag));
    *p++ = 0x00;
    *p++ = 0x74, *p++ = 0x0E;                          // je +14: skip the store and the count
    *p++ = 0xC7, *p++ = 0x46, *p++ = 0x64;             // mov dword [rsi+64h],3F800000h (1.0f)
    put32(0x3F800000);
    *p++ = 0xF0, *p++ = 0xFF, *p++ = 0x05;             // lock inc dword [rip+count]
    put32(rel(p + 4, count));
    // in-step hook: on_record(contact = rsi, car A = rdi, car B = rbx) on the physics thread. Align the stack, keep every
    // volatile register the handler may still use (rax, rdx, r8-r11, xmm0-5).
    for (uint8_t b : {0x55, 0x48, 0x89, 0xE5, 0x48, 0x83, 0xE4, 0xF0, 0x48, 0x81, 0xEC, 0x00, 0x01, 0x00, 0x00})
        *p++ = b;                                      // push rbp; mov rbp,rsp; and rsp,-16; sub rsp,100h
    const uint8_t gp_save[6][5] = {{0x48, 0x89, 0x44, 0x24, 0x20}, {0x48, 0x89, 0x54, 0x24, 0x28},
                                   {0x4C, 0x89, 0x44, 0x24, 0x30}, {0x4C, 0x89, 0x4C, 0x24, 0x38},
                                   {0x4C, 0x89, 0x54, 0x24, 0x40}, {0x4C, 0x89, 0x5C, 0x24, 0x48}};
    for (auto &i : gp_save)
        for (uint8_t b : i)
            *p++ = b;                                  // mov [rsp+20h..48h],rax/rdx/r8/r9/r10/r11
    for (int i = 0; i < 6; i++) {                      // vmovaps [rsp+50h+16i],xmm_i
        *p++ = 0xC5, *p++ = 0xF8, *p++ = 0x29, *p++ = uint8_t(0x84 | (i << 3)), *p++ = 0x24;
        put32(0x50 + 16 * i);
    }
    for (uint8_t b : {0x48, 0x89, 0xF1, 0x48, 0x89, 0xFA, 0x49, 0x89, 0xD8})
        *p++ = b;                                      // mov rcx,rsi; mov rdx,rdi; mov r8,rbx
    *p++ = 0x48, *p++ = 0xB8, put64(reinterpret_cast<uint64_t>(&on_record)); // mov rax,on_record
    *p++ = 0xFF, *p++ = 0xD0;                          // call rax
    for (int i = 0; i < 6; i++) {                      // vmovaps xmm_i,[rsp+50h+16i]
        *p++ = 0xC5, *p++ = 0xF8, *p++ = 0x28, *p++ = uint8_t(0x84 | (i << 3)), *p++ = 0x24;
        put32(0x50 + 16 * i);
    }
    const uint8_t gp_load[6][5] = {{0x48, 0x8B, 0x44, 0x24, 0x20}, {0x48, 0x8B, 0x54, 0x24, 0x28},
                                   {0x4C, 0x8B, 0x44, 0x24, 0x30}, {0x4C, 0x8B, 0x4C, 0x24, 0x38},
                                   {0x4C, 0x8B, 0x54, 0x24, 0x40}, {0x4C, 0x8B, 0x5C, 0x24, 0x48}};
    for (auto &i : gp_load)
        for (uint8_t b : i)
            *p++ = b;
    *p++ = 0x48, *p++ = 0x89, *p++ = 0xEC, *p++ = 0x5D; // mov rsp,rbp; pop rbp
    *p++ = 0x48, *p++ = 0xB9;                          // mov rcx,slot
    put64(slot);
    *p++ = 0x48, *p++ = 0x8B, *p++ = 0x09;             // mov rcx,[rcx]  (= the original mov rcx,[rip+slot])
    *p++ = 0xFF, *p++ = 0x25, put32(0);                // jmp [rip+0] -> hit+7
    put64(hit + 7);
    FlushInstructionCache(GetCurrentProcess(), m, 4096);
    g_eq_flag = flag, g_eq_count = count;
    *g_eq_flag = 0;
    g_log_flag = lflag, g_log_next = lnext, g_ring = reinterpret_cast<Rec *>(ring);
    *g_log_flag = 1;                                   // the contact spin reads the ring too
    uint8_t jump[7] = {0xE9, 0, 0, 0, 0, 0x90, 0x90};  // jmp stub; nop; nop (7 bytes, as the mov)
    int32_t r = int32_t(reinterpret_cast<intptr_t>(m) - intptr_t(hit + 5));
    memcpy(jump + 1, &r, 4);
    return kizi::patch_code(hit, jump, 7);
}

uintptr_t slot_of(const char *pattern, const char *what)
{
    uintptr_t hit = kizi::scan_code(pattern, what);
    int32_t disp = 0;
    return hit && kizi::read(hit + 3, disp) ? hit + 7 + disp : 0;
}

int push_index()
{
    if (sim::cfg::active())
        return 0;                                  // Realistic: the rolling-tyre rub replaces the game's wheel push
    int k = g_push_key;
    if (k >= 0)
        return k;
    std::string_view v = wheel_push.get();
    for (int i = 1; i < LEVELS; i++)
        if (v == PUSH[i].name)
            return i;
    return 0;
}

bool soft_off()
{
    int k = g_soft_key;
    return k >= 0 ? k == 1 : softening.get() == "off";
}

void apply()
{
    if (g_params) {
        int i = push_index();
        float now[3] = {};
        if ((i || g_wrote_push) && kizi::read(g_params, now, sizeof now) &&
            (now[0] != PUSH[i].grip || now[1] != PUSH[i].scale || now[2] != PUSH[i].cap)) {
            float want[3] = {PUSH[i].grip, PUSH[i].scale, PUSH[i].cap};
            if (kizi::write(g_params, want, sizeof want)) {
                g_wrote_push = i != 0;
                swofty::log("[ai-tweaks] wheel-to-wheel push: %s", PUSH[i].name);
            }
        }
    }
    bool off = soft_off();
    for (uintptr_t slot : g_slot) {
        uint64_t node = 0;
        uint8_t on = 0, want = off ? 0 : 1;
        if (!slot || !kizi::read(slot, node) || node < 0x10000 || !kizi::read(node + 8, on) || on > 1)
            continue;                              // the game has not loaded its contact settings yet
        if (on != want && (off || g_wrote_soft) && kizi::write(node + 8, &want, 1)) {
            g_wrote_soft = off;
            swofty::log("[ai-tweaks] contact softening %s", off ? "off" : "back to the game's");
        }
    }
}

// --- contact log ---------------------------------------------------------------------------------------------------
// The handler runs on every physics step while two cars touch; the worker groups those into one "touch" per pair
// (over after 0.3 s without a record). From the touch's first record it samples both cars every 50 ms for 1 s and
// logs, per car, the sideways speed (in the car's own frame) and the spin rate before the touch and their peaks
// after it, plus the front wing damage change. Car body (as the telemetry motion packet reads it):
// [[vehicle+0x558]+0x340]+0x18: +0x1b0 forward direction, +0x190 left direction (the packet's right = minus this),
// +0x220 velocity. Human car: collision mass factor (physics +0x3e4) 50, AI 45 (game database).
uintptr_t g_vehicles;                              // vehicle table (index = vehicle id), for car numbers
uint32_t g_wing_field;                             // vehicle + this = front wing (left / right damage +48h / +4Ch)

struct V3 {
    float x, y, z;
};
float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float len(V3 a) { return std::sqrt(dot(a, a)); }

struct Sample {
    V3 fwd{}, left{}, vel{};
    float wl = 0, wr = 0;
    bool ok = false;
};
Sample sample(uint64_t veh)
{
    Sample s;
    uint64_t phys = 0, rb = 0, body = 0, wing = 0;
    float f[3], l[3], v[3];
    if (!veh || !kizi::read(veh + 0x558, phys) || !phys || !kizi::read(phys + 0x340, rb) || !rb ||
        !kizi::read(rb + 0x18, body) || !body || !kizi::read(body + 0x1b0, f, sizeof f) ||
        !kizi::read(body + 0x190, l, sizeof l) || !kizi::read(body + 0x220, v, sizeof v))
        return s;
    s.fwd = {f[0], f[1], f[2]}, s.left = {l[0], l[1], l[2]}, s.vel = {v[0], v[1], v[2]}, s.ok = true;
    if (g_wing_field && kizi::read(veh + g_wing_field, wing) && wing)
        kizi::read(wing + 0x48, s.wl), kizi::read(wing + 0x4c, s.wr);
    return s;
}
float side_kmh(const Sample &s) { return std::fabs(dot(s.vel, s.left)) * 3.6f; }
// Spin rate between two samples dt ms apart (degrees per second), from the change of the forward direction.
float spin(const Sample &a, const Sample &b, float dt_ms)
{
    float c = std::clamp(dot(a.fwd, b.fwd) / std::max(1e-6f, len(a.fwd) * len(b.fwd)), -1.0f, 1.0f);
    return std::acos(c) * 57.2958f / (dt_ms / 1000.0f);
}
bool human(uint64_t veh)
{
    uint64_t phys = 0;
    float m = 0;
    return veh && kizi::read(veh + 0x558, phys) && phys && kizi::read(phys + 0x3e4, m) && m > 47.5f;
}
int car_id(uint64_t veh)
{
    for (int i = 0; g_vehicles && i < 32; i++) {
        uint64_t v = 0;
        if (kizi::read(g_vehicles + i * 8, v) && v == veh)
            return i;
    }
    return -1;
}

struct Watch {                                     // one car during a touch
    uint64_t veh;
    bool you;
    Sample first, last;
    DWORD last_at;
    float side0, side_peak = 0, spin0 = -1, spin_peak = 0;
};
struct Touch {
    Watch c[2];
    DWORD first, last;
    int steps = 0;
    float ratio_min = 1e9f, ratio_max = 0, closing = 0;
};
std::map<std::pair<uint64_t, uint64_t>, Touch> g_touch;
uint32_t g_read;                                   // ring entries read
int g_ai_ai, g_ai_you;                             // touches in the last 30 s

void log_contacts()
{
    if (!g_ring || !g_log_next)
        return;
    uint32_t end = *g_log_next;
    if (end - g_read > RING)
        g_read = end - RING;                       // fell behind: skip the overwritten part
    DWORD now = GetTickCount();
    for (; g_read != end; g_read++) {
        Rec r = g_ring[g_read & (RING - 1)];
        if (!r.a || !r.b)
            continue;
        auto key = r.a < r.b ? std::make_pair(r.a, r.b) : std::make_pair(r.b, r.a);
        auto it = g_touch.find(key);
        if (it == g_touch.end()) {
            Touch t;
            t.first = now;
            uint64_t v[2] = {key.first, key.second};
            for (int i = 0; i < 2; i++) {
                Watch &w = t.c[i];
                w.veh = v[i], w.you = human(v[i]), w.first = w.last = sample(v[i]), w.last_at = now;
                w.side0 = side_kmh(w.first);
            }
            if (t.c[0].first.ok && t.c[1].first.ok) {
                V3 a = t.c[0].first.vel, b = t.c[1].first.vel;
                t.closing = len({a.x - b.x, a.y - b.y, a.z - b.z}) * 3.6f;
            }
            it = g_touch.emplace(key, t).first;
        }
        Touch &t = it->second;
        t.last = now, t.steps++;
        t.ratio_min = std::min(t.ratio_min, r.ratio), t.ratio_max = std::max(t.ratio_max, r.ratio);
    }
    for (auto it = g_touch.begin(); it != g_touch.end();) {
        Touch &t = it->second;
        for (auto &w : t.c) {                      // sample both cars on every tick for 1 s after the touch began
            if (now - t.first > 1000 || now - w.last_at < 40)
                continue;
            Sample s = sample(w.veh);
            if (!s.ok || !w.last.ok)
                continue;
            float sp = spin(w.last, s, float(now - w.last_at));
            if (w.spin0 < 0)
                w.spin0 = sp;                      // first interval: the car's spin rate as the touch begins
            w.spin_peak = std::max(w.spin_peak, sp);
            w.side_peak = std::max(w.side_peak, side_kmh(s));
            w.last = s, w.last_at = now;
        }
        if (now - t.first < 1000 || now - t.last < 300) {
            ++it;
            continue;
        }
        auto car = [](const Watch &w) {
            char b[200];
            snprintf(b, sizeof b, "%s(%d) sideways %.1f -> peak %.1f km/h, spin %.0f -> peak %.0f deg/s, wing +%.0f/+%.0f",
                     w.you ? "YOU" : "AI", car_id(w.veh), w.side0, w.side_peak, std::max(0.0f, w.spin0), w.spin_peak,
                     w.last.wl - w.first.wl, w.last.wr - w.first.wr);
            return std::string(b);
        };
        bool you = t.c[0].you || t.c[1].you;
        int pk = preset_index();
        std::string_view pn = index_name(pk);
        swofty::log("[ai-tweaks] contact %s [%.*s]: closing %.1f km/h, %d step(s) over %lu ms, game ratio %.2f..%.2f | %s | %s",
                    you ? "YOU-AI" : "AI-AI", int(pn.size()), pn.data(), t.closing, t.steps, t.last - t.first, t.ratio_min,
                    t.ratio_max, car(t.c[0]).c_str(), car(t.c[1]).c_str());
        float sp = std::max(t.c[0].spin_peak, t.c[1].spin_peak);
        if (sp < 1000.0f) {
            PresetStats &ps = g_pstats[pk];
            ps.logged++;
            add_sum(ps.side_sum, std::max(t.c[0].side_peak - t.c[0].side0, t.c[1].side_peak - t.c[1].side0));
            add_max(ps.spin_max, sp);
        }
        (you ? g_ai_you : g_ai_ai)++;
        it = g_touch.erase(it);
    }
}

// --- contact spin ------------------------------------------------------------------------------------------------
// The AI's steering correction acts on every physics step with perfect reactions, so a hit that would rotate a human's
// car hardly turns an AI car (seen in the contact log: you up to ~116 deg/s, the AI ~30). On a touch's first record
// this adds the yaw a real hit gives to every AI car in it: from the contact point and normal (copied by the stub)
// the push on the car is taken as the part of the cars' relative speed along the normal, and its lever about the
// car's centre gives the extra spin: d_omega = level x 0.3 / m^2 x (r x J).up, capped at 2.5 rad/s. The car body
// keeps the angular velocity at +0x210 (the yaw rate +0x214 matched the measured turn rate, 0.97) and its position
// at +0x1c0 (the same fields the game's own apply-impulse function writes). One kick is not enough: the AI counter-
// steers within a few physics steps (the cars' mass, inertia and spin limits are the same for you and the AI, measured).
// So the extra spin is HELD: for HOLD_MS after the touch the car's yaw rate is kept at least at (its rate at the touch
// + the kick fading out linearly), checked every millisecond - the AI cannot correct it away before the car turns.
// Smallest extra yaw a hit AI car gets (rad/s): a push that lands where it hardly turns the car (e.g. straight into
// the rear wheel) still gives a visible twitch, in the direction the push turns it.
bool realistic() { return sim::cfg::active(); }

float twitch_min()
{
    if (realistic())
        return 0.0f;                               // Classic only
    std::string_view v = twitch.get();
    return v == "small" ? 0.25f : v == "medium" ? 0.45f : 0.0f;
}

float spin_level()
{
    if (realistic())
        return 0.0f;
    std::string_view v = contact_spin.get();
    return v == "light" ? 0.35f : v == "medium" ? 0.7f : v == "strong" ? 1.0f : 0.0f;
}

// Smallest extra yaw (rad/s) for the car AHEAD when it is hit behind its centre (a hit on the rear tyre), where the
// push turns it hardest in a real crash.
float rear_min()
{
    if (realistic())
        return 0.0f;
    std::string_view v = rear_boost.get();
    return v == "small" ? 0.5f : v == "medium" ? 0.9f : v == "strong" ? 1.4f : 0.0f;
}

// Upward speed (m/s per unit of tilted-contact closing speed) given to the car behind when its tyre rides up on the
// other car: the contact normal then points partly up.
float lift_gain()
{
    if (realistic())
        return 0.0f;
    std::string_view v = wheel_lift.get();
    return v == "small" ? 1.5f : v == "medium" ? 3.0f : v == "strong" ? 5.0f : 0.0f;
}

// --- crash feed (1.1.1) --------------------------------------------------------------------------------------------
// With "Testing: log contacts" on, the last crash events show in small text in the top-left corner for 6 s each (a rear
// break with its kick and snap, a launch, a flight with its height and time in the air, a hard landing, a puncture), so a
// test shows at once whether they happened. Lines come from the physics thread and the workers; the overlay paints them.
struct FeedLine {
    DWORD t;
    char text[56];
};
std::mutex g_feed_mx;
FeedLine g_feed[4];
unsigned g_feed_next;
void feed(const char *fmt, ...)
{
    if (!contact_log.get())
        return;
    FeedLine line{GetTickCount() | 1, {}};
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line.text, sizeof line.text, fmt, ap);
    va_end(ap);
    for (char &c : line.text)
        c = char(c >= 'a' && c <= 'z' ? c - 32 : c);   // the overlay font has capitals only
    std::lock_guard<std::mutex> lock(g_feed_mx);
    g_feed[g_feed_next++ & 3] = line;
}
struct CarTag {
    char s[12];
};
CarTag car_tag(uint64_t veh)                      // "YOU" or "AI 12"
{
    CarTag t{};
    if (human(veh))
        snprintf(t.s, sizeof t.s, "YOU");
    else
        snprintf(t.s, sizeof t.s, "AI %d", car_id(veh));
    return t;
}

// --- punctures from tyre contact ---------------------------------------------------------------------------------
// The game punctures a tyre in a fixed sequence (seen at its own collision / wear call sites): tyre object W (the
// four wheel objects are [[vehicle+0x580]+0x70+8i], +0x2f0 = world position), W+0x20 flag byte (bit 0/1 = already
// handled / in the puncture list), mgr = [[[W+0x10]+0xd8]+0x3f70], vehicle id = byte [[W+0x10]+0xe8];
// ok = AddPuncture(mgr, id, W+8); on success W+0x20 |= 2, [W+0x98] = max(1.0, [W+0x98]), [W+0x34] = [W+0x38].
// AddPuncture locks its own mutex, so it can be called from the physics thread. We run the same sequence for the wheel
// a touch happens at.
using AddPunctureFn = uint8_t (*)(void *mgr, uint32_t vehicle, void *tyre);
AddPunctureFn g_add_puncture;
std::atomic<int> g_punctures{0};

float puncture_chance()
{
    std::string_view v = puncture.get();
    return v == "rare" ? 0.25f : v == "sometimes" ? 0.5f : v == "often" ? 0.85f : 0.0f;   // 0.9.6: harsher
}

uint32_t g_rng = 2463534242u;
float rnd01()
{
    g_rng ^= g_rng << 13, g_rng ^= g_rng >> 17, g_rng ^= g_rng << 5;
    return float(g_rng & 0xFFFFFF) / float(0x1000000);
}

void rear_hit(uint64_t victim, uint64_t striker, const float pt[3], float impact);   // below (suspension section)

// Roll a puncture for the wheel of `veh` nearest the contact point (only if it is really at a tyre).
bool puncture_wheel(uint64_t veh, const float point[3], float chance)
{
    uint64_t set = 0;
    if (!g_add_puncture || !kizi::read(veh + 0x580, set) || !set)
        return false;
    uint64_t best = 0;
    float best_d = 0.45f;                          // tyre radius ~0.36 m: the point has to be at the tyre
    for (int i = 0; i < 4; i++) {
        uint64_t w = 0;
        float pos[3];
        if (!kizi::read(set + 0x70 + 8 * i, w) || !w || !kizi::read(w + 0x2f0, pos, 12))
            continue;
        float d = len({pos[0] - point[0], pos[1] - point[1], pos[2] - point[2]});
        if (d < best_d)
            best_d = d, best = w;
    }
    if (!best || rnd01() >= chance)
        return false;
    uint8_t flags = 0, id = 0;
    uint64_t owner = 0, a = 0, mgr = 0;
    if (!kizi::read(best + 0x20, flags) || (flags & 3) || !kizi::read(best + 0x10, owner) || !owner ||
        !kizi::read(owner + 0xd8, a) || !a || !kizi::read(a + 0x3f70, mgr) || !mgr || !kizi::read(owner + 0xe8, id))
        return false;
    if (!g_add_puncture(reinterpret_cast<void *>(mgr), id, reinterpret_cast<void *>(best + 8)))
        return false;
    float level = 0, now_ = 0, to = 0;
    kizi::read(best + 0x98, level), kizi::read(best + 0x38, now_);
    flags |= 2;
    to = std::max(1.0f, level);
    kizi::write(best + 0x20, &flags, 1);
    kizi::write(best + 0x98, &to, 4);
    kizi::write(best + 0x34, &now_, 4);
    g_punctures++;
    feed("%s PUNCTURE", car_tag(veh).s);
    return true;
}

using ApplyImpulseFn = void (*)(void *body, const float *impulse, const float *point);   // the game's apply-impulse
ApplyImpulseFn g_apply_impulse;

// Is the contact point within `r` metres of one of the vehicle's four wheel objects ([[veh+0x580]+0x70+8i], +0x2f0 = position)?
bool wheel_near(uint64_t veh, const float point[3], float r)
{
    uint64_t set = 0;
    if (!kizi::read(veh + 0x580, set) || !set)
        return false;
    for (int i = 0; i < 4; i++) {
        uint64_t w = 0;
        float pos[3];
        if (kizi::read(set + 0x70 + 8 * i, w) && w && kizi::read(w + 0x2f0, pos, 12) &&
            len({pos[0] - point[0], pos[1] - point[1], pos[2] - point[2]}) < r)
            return true;
    }
    return false;
}

uint64_t body_of(uint64_t veh)
{
    uint64_t phys = 0, rb = 0, body = 0;
    return veh && kizi::read(veh + 0x558, phys) && phys && kizi::read(phys + 0x340, rb) && rb &&
                   kizi::read(rb + 0x18, body)
               ? body
               : 0;
}

// --- AI reaction after a hit (in AI Driver Behaviour since 0.8.0) ---------------------------------------------------
// How the AI drives (its reaction after a hit and to a slide) is AI Driver Behaviour's job from 0.8.0 (its 0.2.0); this
// addon only finds the hits and tells it, through the function that DLL exports. Without that addon: no reaction hold.
using AiHitFn = void (*)(int idx, float impact);
std::atomic<AiHitFn> g_ai_hit{nullptr};
void find_ai_hit()
{
    if (!g_ai_hit.load())
        if (HMODULE m = GetModuleHandleA("kizi_ai_driver_behaviour.dll"))
            if (auto f = reinterpret_cast<AiHitFn>(GetProcAddress(m, "kizi_ai_hit"))) {
                g_ai_hit = f;
                swofty::log("[ai-tweaks] AI reaction after a hit: handled by AI Driver Behaviour");
            }
}
using AiBrokenFn = void (*)(int idx);           // AI Driver Behaviour 0.3.1: "this AI car's suspension just broke"
std::atomic<AiBrokenFn> g_ai_broken{nullptr};
void broken(int idx)
{
    if (!g_ai_broken.load())
        if (HMODULE m = GetModuleHandleA("kizi_ai_driver_behaviour.dll"))
            g_ai_broken = reinterpret_cast<AiBrokenFn>(GetProcAddress(m, "kizi_ai_broken"));
    if (AiBrokenFn f = g_ai_broken.load(); f && idx >= 0)
        f(idx);
}
void stun(int idx, float impact)
{
    if (AiHitFn f = g_ai_hit.load(); f && idx >= 0)
        f(idx, impact);
}

constexpr DWORD HOLD_MS = 250;
struct Hold {
    uint64_t body;
    float base, kick;                              // yaw rate at the touch, extra yaw (rad/s, signed)
    DWORD start;
};
std::atomic<int> g_spins{0};                       // spins given (logged every 30 s)
std::atomic<int> g_lifts{0};                       // wheel lifts given (logged every 30 s)
std::atomic<float> g_lift_max{0}, g_ny_max{0};     // largest lift (m/s), largest |normal.y| seen in a touch
std::atomic<float> g_spin_max{0};

std::mutex g_spin_mx;                              // holds / seen: the physics thread and the hold worker share them
std::vector<Hold> g_holds;
// Snaps after a rear break (1.1.0, see snap() below), held by the same worker
struct Snap {
    uint64_t body;
    float peak;                                    // rad/s, signed
    DWORD start;
};
std::vector<Snap> g_snaps;
// Flights (1.1.1). An F1 car's downforce comes mostly from its floor, which only works a few centimetres off the road: a car
// thrown up at speed (a wheel climbing a wheel, a hop over a nose or sidepod, a kick) loses it and flies on a plain gravity
// curve (nose up the floor even lifts). The game keeps pressing the car down in the air (up to ~4 g at 250 km/h), so in 1.0 /
// 1.1.0 a 3 m/s pop was back on the road within 0.1 s, a few centimetres up. Now every car in a touch above 100 km/h (AI, and
// yours with "your car too") is watched for 0.4 s. It flies when our own impulses threw it up by 1 m/s or more in total (the
// tyre rub of kizi_contact_sim.hpp, a launch, a kick: they report it) or the game's did by 1.5 m/s along the car's own up
// axis (a hill moves a car along its floor, so a crest or a dip never counts). Its vertical speed (body +0x224) is then kept
// on the level's gravity curve from the pop (starting at most the level's cap above its speed before the touch) until it
// lands, at most 2.5 s, by the same 1 ms worker as the spins: only raised back to the curve, never lowered, so the game's own
// physics (pitch, roll, hitting things) stays. A car that is moved (flashback, restart, reset) leaves the watch.
struct Fly {
    uint64_t veh, body;
    float base, anchor;                            // vertical speed before the touch; where the curve starts (m/s)
    double anchor_s;                               // when the curve starts (steady clock, s)
    float kmh, pop, kick, y0, y_max, last_pos[3];   // kick: upward speed our own impulses gave it while watched
    DWORD armed, start, last_t;
    int held;                                      // ticks that put the car back on the curve
    bool flying;
};
std::vector<Fly> g_flies;
std::atomic<int> g_flight_n{0}, g_fly_held{0};
std::atomic<float> g_fly_high{0}, g_fly_long{0};
double clock_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

void arm_flight(uint64_t veh, DWORD now)            // on_record, g_spin_mx held
{
    uint64_t b = body_of(veh);
    float v[3], p[3];
    if (!b || !kizi::read(b + 0x220, v, 12) || !kizi::read(b + 0x1c0, p, 12) || len({v[0], v[1], v[2]}) < 27.8f)
        return;                                    // under 100 km/h
    for (auto &f : g_flies)
        if (f.body == b) {
            if (!f.flying)
                f.armed = now, f.base = v[1];      // a new touch while still watching: watch again from here
            return;
        }
    Fly f{};
    f.veh = veh, f.body = b, f.base = v[1], f.armed = f.last_t = now, f.kmh = len({v[0], v[1], v[2]}) * 3.6f;
    memcpy(f.last_pos, p, 12);
    g_flies.push_back(f);
}

void lift_noted(uint64_t body, float dv)            // our impulse threw this car up by dv m/s; g_spin_mx held
{
    for (auto &f : g_flies)
        if (f.body == body && !f.flying)
            f.kick += dv;
}

// One 1 ms tick of every watched car (spin_worker, g_spin_mx held). Finished flights go to `done` (logged after the lock).
void fly_tick(DWORD t, std::vector<std::pair<Fly, bool>> &done)
{
    if (g_flies.empty())
        return;
    Crash c = crash();
    double now_s = clock_s();
    for (auto it = g_flies.begin(); it != g_flies.end();) {
        Fly &f = *it;
        float v[3], p[3];
        bool ok = kizi::read(f.body + 0x220, v, 12) && kizi::read(f.body + 0x1c0, p, 12) && std::isfinite(v[1]) &&
                  std::isfinite(p[1]);
        float dt = float(t - f.last_t) / 1000.0f;
        bool moved = ok && len({p[0] - f.last_pos[0], p[1] - f.last_pos[1], p[2] - f.last_pos[2]}) >
                               5.0f + len({v[0], v[1], v[2]}) * dt * 2.0f;
        if (!ok || moved || (!f.flying && t - f.armed > 400)) {
            it = g_flies.erase(it);                // gone, moved, or it never popped
            continue;
        }
        f.last_t = t;
        memcpy(f.last_pos, p, 12);
        if (!f.flying) {
            float up[3] = {};
            bool kicked = f.kick >= 1.0f && v[1] - f.base >= 0.5f;
            bool popped = v[1] - f.base >= 1.5f && kizi::read(f.body + 0x1a0, up, 12) &&
                          v[0] * up[0] + v[1] * up[1] + v[2] * up[2] >= 1.0f;
            if (kicked || popped) {
                float from = kicked ? std::max(v[1], f.base + 0.9f * f.kick) : v[1];
                f.flying = true, f.start = t, f.anchor_s = now_s, f.anchor = std::min(from, f.base + c.fly_cap);
                f.y0 = f.y_max = p[1], f.pop = v[1] - f.base;
            }
            ++it;
            continue;
        }
        f.y_max = std::max(f.y_max, p[1]);
        f.pop = std::max(f.pop, v[1] - f.base);
        float want = f.anchor - c.fly_g * float(now_s - f.anchor_s);
        bool falling = want < f.base;              // past the top of the curve
        DWORD age = t - f.start;
        if (falling && v[1] > want + 1.5f && age > 120) {
            done.push_back({f, true});             // the road (or a car) stopped the fall: landed
            it = g_flies.erase(it);
            continue;
        }
        if (!falling && age < 150 && v[1] > want + 0.05f)   // still being pushed up (the rub, the suspension): follow it
            f.anchor = std::min(v[1], f.base + c.fly_cap), f.anchor_s = now_s;
        else if (v[1] < want - 0.02f && kizi::write(f.body + 0x224, &want, 4))
            f.held++;                              // pressed down harder than gravity: back on the curve
        if (age > 2500 || want < f.base - 12.0f) {
            done.push_back({f, false});
            it = g_flies.erase(it);
            continue;
        }
        ++it;
    }
}

void flight_done(const Fly &f, bool landed)        // spin_worker, after the lock
{
    float secs = float(f.last_t - f.start) / 1000.0f, high = f.y_max - f.y0;
    g_flight_n++;
    g_fly_held += f.held;
    add_max(g_fly_high, high);
    add_max(g_fly_long, secs);
    PresetStats &ps = g_pstats[preset_index()];
    ps.flights++;
    add_max(ps.fly_max, high);
    if (high >= 0.5f || (contact_log.get() && high >= 0.05f))
        swofty::log("[ai-tweaks] flight: car %d (%s) popped %.1f m/s at %.0f km/h -> %.2f m high, %.2f s in the air%s, put back on "
                    "the curve %d time(s) (crashes %.*s)", car_id(f.veh), human(f.veh) ? "YOU" : "AI", double(f.pop),
                    double(f.kmh), double(high), double(secs), landed ? "" : " (no landing seen)", f.held,
                    int(level_name().size()), level_name().data());
    if (high >= 0.15f)
        feed("%s FLEW %.1f M HIGH  %.1f S", car_tag(f.veh).s, double(high), double(secs));
}
// Slides (1.2.0). In the game a sliding or spinning car stops far too soon. In the 1.1.1 log a broken AI car went from 136
// km/h to a standstill in 2 s (about 1.9 g: once the game declares it terminally damaged it locks it to a stop, while the AI
// itself still asks for full throttle), and cars sliding sideways (slip up to 104 deg) slowed just as hard: the game's tyres
// keep nearly full grip at any slip angle. Real slicks sliding sideways at speed keep roughly half of it (rubber grips less
// the faster it slides, and sideways the aero is gone), so a spun F1 car slides on for 100 m or more and keeps rotating
// (Russell, Silverstone 2022), and an upside-down one skates along on its roll hoop (Zhou, same crash). The slide hold
// follows a car (AI, and yours with "your car too") from 35 deg of slip at 50 km/h (seen by the 20 Hz aero check), on its
// side or upside down from 30 km/h, and for 4 s after a rear break; it lets go once the car is straight again for 0.3 s
// (slip under 15 deg, yaw under 45 deg/s), under 30 km/h, or after 10 s. Every 1 ms tick gives back the level's share of
// the horizontal speed and of the yaw rate the car lost since the last tick (upside down: 75 % of the speed), so the
// game's own grip, gravel, scraping and brakes still slow it, only less. A sudden loss (a wall, another car: about 4 g
// more than a slide in one tick) is left alone; a car that is moved (flashback, restart, reset) is dropped.
struct Slide {
    uint64_t veh, body;
    double last_s;                                 // steady clock at the last tick (s)
    float sp, w;                                   // horizontal speed (m/s) and yaw rate (rad/s) after the last tick
    float kmh0, dist, turns, sp_given, w_given, last_pos[3];
    DWORD start, end, calm, broken_until;          // calm: straight again since (0 = sliding)
    double win_s;                                  // 1.2.1 replay check: window start, position then, travel by the speed
    float win_pos[2], win_travel[2];
};
std::vector<Slide> g_slides;
std::atomic<int> g_slide_n{0};
std::atomic<float> g_slide_far{0}, g_slide_turns{0};

void arm_slide(uint64_t veh, uint64_t body, DWORD broken_ms)   // g_spin_mx held
{
    DWORD now = GetTickCount();
    for (auto &sl : g_slides)
        if (sl.body == body) {
            if (broken_ms)
                sl.broken_until = (now + broken_ms) | 1;
            return;
        }
    float b[0x9c / 4];                             // body +0x190 .. +0x22c, as aero_compute reads it
    if (!body || !kizi::read(body + 0x190, b, sizeof b) || !std::isfinite(b[36]) || !std::isfinite(b[38]))
        return;
    Slide sl{};
    sl.veh = veh, sl.body = body, sl.start = now, sl.last_s = clock_s();
    sl.sp = std::hypot(b[36], b[38]), sl.w = b[33], sl.kmh0 = sl.sp * 3.6f;
    sl.last_pos[0] = b[12], sl.last_pos[1] = b[13], sl.last_pos[2] = b[14];
    sl.win_s = sl.last_s, sl.win_pos[0] = b[12], sl.win_pos[1] = b[14];
    sl.broken_until = broken_ms ? (now + broken_ms) | 1 : 0;
    g_slides.push_back(sl);
}

// One 1 ms tick of every followed car (spin_worker, g_spin_mx held). Finished slides go to `done` (logged after the lock).
void slide_tick(DWORD t, std::vector<Slide> &done)
{
    if (g_slides.empty())
        return;
    Crash c = crash();
    double now_s = clock_s();
    for (auto it = g_slides.begin(); it != g_slides.end();) {
        Slide &sl = *it;
        float b[0x9c / 4];
        if (!kizi::read(sl.body + 0x190, b, sizeof b) || !std::isfinite(b[36]) || !std::isfinite(b[38]) || !std::isfinite(b[33])) {
            it = g_slides.erase(it);               // gone (car rebuilt, session over)
            continue;
        }
        float dt = std::clamp(float(now_s - sl.last_s), 0.0f, 0.1f);
        float vx = b[36], vz = b[38], sp = std::hypot(vx, vz), w = b[33];
        float moved = std::hypot(b[12] - sl.last_pos[0], b[14] - sl.last_pos[2]);
        if (moved > 5.0f + sp * dt * 2.0f) {       // moved: flashback, restart, reset
            it = g_slides.erase(it);
            continue;
        }
        sl.last_pos[0] = b[12], sl.last_pos[1] = b[13], sl.last_pos[2] = b[14];
        sl.dist += moved, sl.turns += std::fabs(w) * dt / 6.2832f;
        // 1.2.1: a replay or a flashback rewind moves cars without the physics; over 0.1 s the travel then no longer matches the
        // speed (much farther, or backwards: the 1.2.0 log had "190 m in 0.8 s" slides). Such a car is dropped, not logged.
        sl.win_travel[0] += vx * dt, sl.win_travel[1] += vz * dt;
        if (now_s - sl.win_s >= 0.1) {
            float ex = sl.win_travel[0], ez = sl.win_travel[1], dx = b[12] - sl.win_pos[0], dz = b[14] - sl.win_pos[1];
            float el = std::hypot(ex, ez), dl = std::hypot(dx, dz);
            if (dl > 2.0f * el + 1.0f || (el > 2.0f && dx * ex + dz * ez < 0)) {
                it = g_slides.erase(it);
                continue;
            }
            sl.win_s = now_s, sl.win_pos[0] = b[12], sl.win_pos[1] = b[14], sl.win_travel[0] = sl.win_travel[1] = 0;
        }
        float fx = b[8], fz = b[10], fl = std::hypot(fx, fz), ul = len({b[4], b[5], b[6]});
        float slip = fl > 0.3f && sp > 1.0f ? std::acos(std::clamp((vx * fx + vz * fz) / (sp * fl), -1.0f, 1.0f)) * 57.2958f
                                            : 0.0f;
        bool flip = ul > 0.5f && b[5] / ul < 0.2f, broken = sl.broken_until && int(sl.broken_until - t) > 0;
        bool sliding = sp >= 8.3f && (slip >= 15.0f || std::fabs(w) >= 0.8f || flip || broken);
        if (!sliding || int(t - sl.start) > 10000) {
            if (!sl.calm)
                sl.calm = t ? t : 1;
            if (sp < 8.3f || int(t - sl.start) > 10000 || int(t - sl.calm) > 300) {
                sl.end = t;
                done.push_back(sl);
                it = g_slides.erase(it);
                continue;
            }
        } else {
            sl.calm = 0;
            float keep = flip ? 0.75f : c.slide_keep, lost = sl.sp - sp;
            if (lost > 0 && lost < 0.3f + 40.0f * dt && sp > 0.5f) {   // grip, gravel, scraping, brakes - not an impact
                float k = (sp + keep * lost) / sp, nx = vx * k, nz = vz * k;
                if (kizi::write(sl.body + 0x220, &nx, 4) && kizi::write(sl.body + 0x228, &nz, 4))
                    sl.sp_given += keep * lost, sp += keep * lost;
            }
            float wl = std::fabs(sl.w) - std::fabs(w);
            if (!flip && wl > 0 && w * sl.w > 0 && wl < 2.0f + 60.0f * dt) {   // the spin slowing down - not an impact
                float nw = w + (w > 0 ? c.spin_keep : -c.spin_keep) * wl;
                if (kizi::write(sl.body + 0x214, &nw, 4))
                    sl.w_given += c.spin_keep * wl, w = nw;
            }
        }
        sl.sp = sp, sl.w = w, sl.last_s = now_s;
        ++it;
    }
}

void slide_done(const Slide &sl)                   // spin_worker, after the lock
{
    float secs = float(sl.end - sl.start) / 1000.0f;
    g_slide_n++;
    add_max(g_slide_far, sl.dist);
    add_max(g_slide_turns, sl.turns);
    PresetStats &ps = g_pstats[preset_index()];
    ps.slides++;
    add_max(ps.slide_max, sl.dist);
    if (sl.dist >= 60.0f || (contact_log.get() && sl.dist >= 15.0f))
        swofty::log("[ai-tweaks] slide: car %d (%s) from %.0f km/h%s: %.0f m in %.1f s, %.1f turn(s); given back %.0f km/h of speed "
                    "and %.0f deg/s of spin (crashes %.*s)", car_id(sl.veh), human(sl.veh) ? "YOU" : "AI", double(sl.kmh0),
                    sl.broken_until ? " (rear broken)" : "", double(sl.dist), double(secs), double(sl.turns),
                    double(sl.sp_given * 3.6f), double(sl.w_given * 57.2958f), int(level_name().size()), level_name().data());
    if (sl.dist >= 25.0f)
        feed("%s SLID %.0f M  %.1f S  %.1f TURNS", car_tag(sl.veh).s, double(sl.dist), double(secs), double(sl.turns));
}
std::map<std::pair<uint64_t, uint64_t>, DWORD> g_seen;   // pair -> last record time (a touch = no gap > 300 ms)

// Runs INSIDE the contact handler, on the game's physics thread, for every record (the stub calls it). The kick is
// added there, in the same step and the same place the game's own impulse function (apply impulse: body +0x210
// angular velocity, +0x220 velocity) lands, so the physics step cannot overwrite it the way it did a write from our
// own thread between steps. The hold below then keeps it alive while the AI tries to correct.
void on_record(uint64_t contact, uint64_t a, uint64_t b)
{
    sim::CostTimer cost_timer;                     // counted with the contact physics' own cost (logged every 30 s)
    if (!a || !b)
        return;
    float level = spin_level();
    DWORD now = GetTickCount();
    std::lock_guard<std::mutex> lock(g_spin_mx);
    auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    auto it = g_seen.find(key);
    bool fresh = it == g_seen.end() || now - it->second > 300;
    g_seen[key] = now;
    if (g_seen.size() > 256)
        for (auto s = g_seen.begin(); s != g_seen.end();)
            s = now - s->second > 2000 ? g_seen.erase(s) : std::next(s);
    if (!fresh)
        return;
    bool ha = human(a), hb = human(b);
    if (ha && hb)
        return;
    if (spin_who.get() == "you" && !ha && !hb)
        return;                                    // AI-AI touch, but only touches with you count
    float impact = 0;                              // closing speed along the contact normal at the first record of the touch
    {
        float nrm[3], v1[3], v2[3];
        uint64_t b1 = body_of(a), b2 = body_of(b);
        if (b1 && b2 && kizi::read(contact + 0x80, nrm, 12) && kizi::read(b1 + 0x220, v1, 12) && kizi::read(b2 + 0x220, v2, 12)) {
            V3 nn{nrm[0], nrm[1], nrm[2]}, rv{v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]};
            float l = len(nn);
            impact = l > 0.5f ? std::fabs(dot(rv, nn)) / l : len(rv);
        }
    }
    {
        PresetStats &ps = g_pstats[preset_index()];
        ps.touches++;
        add_sum(ps.closing_sum, impact);
        add_max(ps.closing_max, impact);
    }
    if (!ha)
        stun(car_id(a), impact);
    if (!hb)
        stun(car_id(b), impact);
    if (realistic()) {                             // 1.1.1: watch both cars for a pop (flights, see Fly)
        if (!ha || your_car.get())
            arm_flight(a, now);
        if (!hb || your_car.get())
            arm_flight(b, now);
    }
    if (float pc = puncture_chance(); pc > 0) {
        float pt[3], va2[3], vb2[3];
        uint64_t b1 = body_of(a), b2 = body_of(b);
        if (b1 && b2 && kizi::read(contact + 0x70, pt, 12) && kizi::read(b1 + 0x220, va2, 12) &&
            kizi::read(b2 + 0x220, vb2, 12)) {
            // 0.9.6: by the cars' speed (from 40 km/h, full at 150) and the hit (from 0.3 m/s closing, full at 2 m/s). Until 0.9.5
            // it went by the difference in speed, only a few km/h in racing contact, so punctures almost never happened.
            float kmh = std::max(len({va2[0], va2[1], va2[2]}), len({vb2[0], vb2[1], vb2[2]})) * 3.6f;
            float c = pc * std::clamp((kmh - 40.0f) / 110.0f, 0.0f, 1.0f) * std::clamp((impact - 0.3f) / 1.7f, 0.0f, 1.0f);
            if (c > 0) {
                if (!ha || puncture_you)
                    puncture_wheel(a, pt, c);
                if (!hb || puncture_you)
                    puncture_wheel(b, pt, c);
            }
        }
    }
    {                                              // 1.2.1: always (the launch); rear_hit checks the break setting itself
        float pt[3];
        if (kizi::read(contact + 0x70, pt, 12)) {
            if (!ha || jump_you)
                rear_hit(a, b, pt, impact);
            if (!hb || jump_you)
                rear_hit(b, a, pt, impact);
        }
    }
    if (level <= 0)
        return;
    float point[3], normal[3];
    uint64_t ba = body_of(a), bb = body_of(b);
    float va[3], vb[3], pa[3], pb[3];
    if (!ba || !bb || !kizi::read(contact + 0x70, point, 12) || !kizi::read(contact + 0x80, normal, 12) ||
        !kizi::read(ba + 0x220, va, 12) || !kizi::read(bb + 0x220, vb, 12) || !kizi::read(ba + 0x1c0, pa, 12) ||
        !kizi::read(bb + 0x1c0, pb, 12))
        return;
    V3 n{normal[0], normal[1], normal[2]};
    float nl = len(n);
    if (nl < 0.5f || nl > 2.0f)
        return;
    n = {n.x / nl, n.y / nl, n.z / nl};
    V3 rel{va[0] - vb[0], va[1] - vb[1], va[2] - vb[2]};
    float rn = dot(rel, n);                        // relative speed along the normal
    if (std::fabs(rn) < 0.3f)
        return;
    V3 ja{-rn * n.x, -rn * n.y, -rn * n.z};        // push on A (against its approach), B gets minus that
    float ny = std::fabs(n.y);
    if (ny > g_ny_max)
        g_ny_max = ny;
    if (float gain = lift_gain(); gain > 0 && ny >= 0.2f && g_apply_impulse) {
        // the car behind (its nose points at the other car) is the one that climbs; the lift is the game's own impulse at
        // the contact point, so a touch at a rear wheel lifts the rear, not the whole car (and only if the point is at
        // one of that car's wheels)
        float fa[3] = {};
        bool a_behind = kizi::read(ba + 0x1b0, fa, 12) &&
                        fa[0] * (pb[0] - pa[0]) + fa[1] * (pb[1] - pa[1]) + fa[2] * (pb[2] - pa[2]) > 0;
        bool h_ = a_behind ? ha : hb;
        uint64_t veh = a_behind ? a : b, body = a_behind ? ba : bb;
        float inv_mass = 0;
        if (!h_ && wheel_near(veh, point, 0.6f) && kizi::read(body + 0x438, inv_mass) && inv_mass > 1e-6f) {
            float dv = std::min(gain * ny * std::min(std::fabs(rn), 15.0f) / 15.0f, 3.0f);
            alignas(16) float J[4] = {0, dv / inv_mass, 0, 0};   // upward impulse giving the body dv m/s
            alignas(16) float P[4] = {point[0], point[1], point[2], 1};
            g_apply_impulse(reinterpret_cast<void *>(body), J, P);
            g_lifts++;
            if (dv > g_lift_max)
                g_lift_max = dv;
        }
    }
    struct One {
        bool human;
        uint64_t body;
        float *pos;
        float sign;
    } cars[2] = {{ha, ba, pa, 1.0f}, {hb, bb, pb, -1.0f}};
    for (auto &c : cars) {
        if (c.human)
            continue;                              // your car keeps the game's own physics
        V3 rr{point[0] - c.pos[0], point[1] - c.pos[1], point[2] - c.pos[2]};
        if (len(rr) > 6.0f)
            continue;                              // point not near this car: skip rather than guess
        float fw[3] = {};
        const float *op = c.pos == pa ? pb : pa;   // the other car's position
        bool ahead = kizi::read(c.body + 0x1b0, fw, 12) &&
                     fw[0] * (op[0] - c.pos[0]) + fw[1] * (op[1] - c.pos[1]) + fw[2] * (op[2] - c.pos[2]) < 0;
        std::string_view who = spin_target.get();
        if ((who == "behind" && ahead) || (who == "ahead" && !ahead))
            continue;
        V3 j{ja.x * c.sign, ja.y * c.sign, ja.z * c.sign};
        float yaw = rr.z * j.x - rr.x * j.z;       // (r x J) . up (y up)
        float dw = std::clamp(level * 0.3f * yaw, -2.5f, 2.5f);
        float tmin = twitch_min();
        if (ahead && fw[0] * rr.x + fw[1] * rr.y + fw[2] * rr.z < -0.3f)
            tmin = std::max(tmin, rear_min());     // hit behind the centre of the car ahead
        if (tmin > 0 && std::fabs(dw) < tmin) {
            float side = yaw != 0 ? yaw : (rr.z * n.x - rr.x * n.z) * c.sign;   // direction of the push
            dw = side >= 0 ? tmin : -tmin;
        }
        float w = 0;
        if (dw != 0 && kizi::read(c.body + 0x214, w)) {
            g_holds.erase(std::remove_if(g_holds.begin(), g_holds.end(),
                                         [&](const Hold &h) { return h.body == c.body; }),
                          g_holds.end());
            g_holds.push_back({c.body, w, dw, now});
            float to = w + dw;
            kizi::write(c.body + 0x214, &to, 4);   // in this physics step
            g_spins++;
            if (std::fabs(dw) > g_spin_max)
                g_spin_max = std::fabs(dw);
        }
    }
}

// Keeps each extra spin alive while it fades (the AI counter-steers within a few physics steps).
void spin_worker()
{
    bool idle = true;
    std::vector<std::pair<Fly, bool>> done;
    std::vector<Slide> slid;
    for (;;) {
        Sleep(idle ? 25 : 1);                      // 1 ms only while a Classic spin, a snap, a flight or a slide is held
        for (auto &d : done)
            flight_done(d.first, d.second);
        done.clear();
        for (auto &sl : slid)
            slide_done(sl);
        slid.clear();
        std::lock_guard<std::mutex> lock(g_spin_mx);
        // 1.1.1: the time is read under the lock. Entries are stamped under it too, so none is newer than t (read before the
        // lock, a snap made meanwhile could look 49 days old and was dropped at once).
        DWORD t = GetTickCount();
        idle = g_holds.empty() && g_snaps.empty() && g_flies.empty() && g_slides.empty();
        fly_tick(t, done);
        slide_tick(t, slid);
        for (auto it = g_snaps.begin(); it != g_snaps.end();) {
            DWORD age = t - it->start;
            if (age >= 900) {
                it = g_snaps.erase(it);
                continue;
            }
            float f = age < 120 ? float(age) / 120.0f : age < 500 ? 1.0f : 1.0f - float(age - 500) / 400.0f;
            float want = it->peak * f, w = 0;
            if (kizi::read(it->body + 0x214, w) && (it->peak > 0 ? w < want : w > want))
                kizi::write(it->body + 0x214, &want, 4);
            ++it;
        }
        for (auto it = g_holds.begin(); it != g_holds.end();) {
            DWORD age = t - it->start;
            if (age >= HOLD_MS) {
                it = g_holds.erase(it);
                continue;
            }
            float want = it->base + it->kick * (1.0f - float(age) / HOLD_MS), w = 0;
            if (kizi::read(it->body + 0x214, w) && (it->kick > 0 ? w < want : w > want))
                kizi::write(it->body + 0x214, &want, 4);
            ++it;
        }
    }
}

// --- aero loss when sideways (0.7.0) ------------------------------------------------------------------------------------
// An F1 car's wings and floor only work with the air coming from the front: sideways or backwards most of the downforce is
// gone, which is why a spinning car slides so far. The worker measures every car's slip angle (between where it points and
// where it goes, on the ground plane) at 20 Hz; above 15 degrees and 12 m/s the car's downforce multiplier (the game's own
// F1Aero lift_scale, through Swofty's live car field aero_lift_scale) is lowered smoothly to the option's floor at 90 degrees
// and more, and put back when the car points where it goes again. Reading happens on the worker thread, setting the field on
// the game's frame thread.
namespace live = swofty::cars::live;

float aero_floor() { return crash().yaw_floor; }

// 1.1.0, attitude. An F1 car's downforce needs the air to meet the floor and wings at a small angle. When the nose points
// up against the direction of travel (a car climbing a wheel or a kerb at speed: angle of attack = car pitch minus the pitch
// of its velocity, so hills don't count) the floor stops working and, past ~12 degrees, starts to LIFT: the real airborne
// crash (Webber Valencia 2010, Le Mans blow-overs). From 5 degrees nose up (above 100 km/h) the downforce multiplier fades
// to the level's lift_min at 12 degrees; on its side (car's up axis below 60 degrees off vertical) it goes to 0.
std::atomic<int> g_blowovers{0};
std::atomic<float> g_aoa_max{0}, g_aoa_kmh{0};

std::atomic<float> g_aero_want[32];
float g_aero_applied[32], g_aero_base[32];
uint64_t g_aero_veh[32], g_aero_body[32];
std::atomic<bool> g_aero_disabled{false};
std::atomic<int> g_aero_lowered{0};
std::atomic<float> g_aero_min{1.0f};
std::atomic<int> g_aero_wanted{0}, g_aero_refused{0};
std::atomic<float> g_aero_beta_max{0};
int g_aero_last_result = -1;

void aero_compute()
{
    if (!g_vehicles || g_aero_disabled)
        return;
    float floor = aero_floor();
    for (int i = 0; i < 32; i++) {
        uint64_t veh = 0;
        float want = 1.0f;
        if (floor < 1.0f && kizi::read(g_vehicles + i * 8, veh) && veh) {
            if (g_aero_body[i] == 0 || g_aero_veh[i] != veh)
                g_aero_veh[i] = veh, g_aero_body[i] = body_of(veh);
            float b[0x9c / 4] = {};                // body +0x190 .. +0x22c: left, up, fwd .. velocity
            if (g_aero_body[i] && kizi::read(g_aero_body[i] + 0x190, b, sizeof b)) {
                V3 up{b[4], b[5], b[6]}, fwd{b[8], b[9], b[10]}, vel{b[(0x220 - 0x190) / 4], b[(0x224 - 0x190) / 4],
                                                                     b[(0x228 - 0x190) / 4]};
                float ul = len(up);
                if (ul > 0.5f) {
                    up = {up.x / ul, up.y / ul, up.z / ul};
                    float vu = dot(vel, up), fu = dot(fwd, up);
                    V3 vh{vel.x - up.x * vu, vel.y - up.y * vu, vel.z - up.z * vu};
                    V3 fh{fwd.x - up.x * fu, fwd.y - up.y * fu, fwd.z - up.z * fu};
                    float sp = len(vh), fl = len(fh), beta = 0;
                    if (sp > 12.0f && fl > 0.3f) {
                        float c = std::clamp(dot(vh, fh) / (sp * fl), -1.0f, 1.0f);
                        beta = std::acos(c) * 57.2958f;                // slip angle, 0 .. 180 degrees
                        float t = std::clamp((beta - 15.0f) / 75.0f, 0.0f, 1.0f);
                        want = floor < 1.0f ? 1.0f - (1.0f - floor) * t * t * (3.0f - 2.0f * t) : 1.0f;
                        if (beta > g_aero_beta_max)
                            g_aero_beta_max = beta;
                        if (want < 1.0f && g_aero_want[i].load() == 1.0f)
                            g_aero_wanted++;       // a slide starts (counted even if the field can't be set)
                    }
                    float speed = len(vel);
                    if (physics_mode.get() == "realistic" && speed > 28.0f) {
                        float fwl = len(fwd);
                        float aoa = fwl > 0.3f ? (std::asin(std::clamp(fwd.y / fwl, -1.0f, 1.0f)) -
                                                  std::asin(std::clamp(vel.y / speed, -1.0f, 1.0f))) * 57.2958f
                                               : 0.0f;
                        float lmin = crash().lift_min;
                        if (aoa > 5.0f) {
                            float t = std::clamp((aoa - 5.0f) / 7.0f, 0.0f, 1.0f);
                            want = std::min(want, 1.0f - (1.0f - lmin) * t);
                            if (aoa > 8.0f && g_aero_want[i].load() > 0.5f) {
                                g_blowovers++;
                                g_pstats[preset_index()].blowovers++;
                            }
                            if (aoa > g_aoa_max)
                                g_aoa_max = aoa, g_aoa_kmh = speed * 3.6f;
                            add_max(g_pstats[preset_index()].aoa_max, aoa);
                        }
                        if (up.y < 0.5f)
                            want = std::min(want, 0.0f);   // on its side or upside down: no downforce
                    }
                    // 1.2.0: sliding (35 deg from 50 km/h), on its side or upside down (from 30 km/h): the slide hold follows it
                    if (((beta >= 35.0f && sp >= 13.9f) || (up.y < 0.2f && speed > 8.3f)) && realistic() &&
                        (!human(veh) || your_car.get())) {
                        std::lock_guard<std::mutex> lock(g_spin_mx);
                        arm_slide(veh, g_aero_body[i], 0);
                    }
                }
            } else
                g_aero_body[i] = 0;
        }
        g_aero_want[i] = want;
    }
}

// Direct path (0.9.1). When Swofty's live car operations are off (the core keeps them off on a game build it has no field
// offsets for, as after the November 2026 update), the downforce multiplier is written straight into the car's F1Aero
// system: CarLive+0x570 (our vehicle pointer is the CarLive), plain float lift_scale at +0x19a4, 1.0 by default, read by the
// physics step every substep (Swofty's research notes, re/db/subsystems/car.md). Before each write the system is checked:
// its vtable is in the game image, its car pointer (+8) is this car, and the value is a sane multiplier.
float *aero_lift_ptr(int i, bool any = false)
{
    uint64_t veh = 0, sys = 0, vt = 0, owner = 0;
    float v = 0;
    if (!kizi::read(g_vehicles + i * 8, veh) || !veh || !kizi::read(veh + 0x570, sys) || !sys || !kizi::read(sys, vt) ||
        !kizi::in_image(vt) || !kizi::read(sys + 8, owner) || owner != veh || !kizi::read(sys + 0x19a4, v) || !std::isfinite(v) ||
        v > 5.0f || v < (any ? -5.0f : 0.02f))     // any: also a value this addon wrote below 0 (lift)
        return nullptr;
    return reinterpret_cast<float *>(sys + 0x19a4);
}

void aero_frame(double, void *)
{
    static double acc = 0;
    static bool checked = false, direct_logged = false;
    acc += 1.0 / 60.0;                             // the frame tick passes dt; a fixed step is enough for a 20 Hz rate
    if (acc < 0.05 || g_aero_disabled || !g_vehicles)
        return;
    acc = 0;
    bool direct = !live::available();
    if (direct && !direct_logged) {
        int found = 0, have = 0;
        for (int i = 0; i < 32; i++) {
            uint64_t veh = 0;
            if (kizi::read(g_vehicles + i * 8, veh) && veh)
                have++, found += aero_lift_ptr(i) != nullptr;
        }
        if (!have)
            return;                                // no cars yet (menus)
        direct_logged = true;
        swofty::log("[ai-tweaks] aero loss: Swofty's live car operations are off on this build - writing the downforce multiplier "
                    "directly (F1Aero +0x19a4): found on %d of %d car(s)", found, have);
        if (!found) {
            swofty::log("[ai-tweaks] aero loss: the F1Aero system did not check out on any car - not used");
            g_aero_disabled = true;
            return;
        }
    }
    if (!direct && !checked) {                                // the live car index must be our vehicle table index (checked once cars exist)
        int match = 0, have = 0;
        for (const auto &c : live::cars()) {
            uint64_t v = 0;
            if (!c.handle() || c.index() >= 32 || !kizi::read(g_vehicles + c.index() * 8, v))
                continue;
            have++;
            match += reinterpret_cast<uint64_t>(c.handle()) == v;
        }
        if (!have)
            return;                                // no cars yet (menus): check again next time
        checked = true;
        swofty::log("[ai-tweaks] aero loss: live car handles %d of %d match the vehicle table", match, have);
        if (have && !match) {
            swofty::log("[ai-tweaks] aero loss: car index mapping unknown - not used");
            g_aero_disabled = true;
            return;
        }
    }
    for (int i = 0; i < 32; i++) {
        float want = g_aero_want[i];
        float &applied = g_aero_applied[i];
        if (applied == 0)
            applied = 1.0f;
        if (std::fabs(want - applied) < 0.04f && !(want == 1.0f && applied != 1.0f))
            continue;
        if (direct || want < 0.0f || applied < 0.0f) {
            // 1.1.0: a negative multiplier (lift, nose up) is written straight to F1Aero +0x19a4: Swofty's aero.lift_scale
            // takes 0 .. 10 only. Coming back from below 0 also goes this way once, then the live field again.
            float *lift = aero_lift_ptr(i, applied < 1.0f);
            if (!lift) {
                applied = 1.0f;                    // car gone or rebuilt: start again from its own value
                continue;
            }
            if (applied == 1.0f && direct)
                g_aero_base[i] = *lift;            // start of a slide: remember the car's own value
            else if (applied == 1.0f) {
                auto b = live::Car{static_cast<uint32_t>(i)}.get(live::Field::aero_lift_scale);
                if (!b)
                    continue;
                g_aero_base[i] = *b;
            }
            float val = want == 1.0f ? g_aero_base[i] : g_aero_base[i] * want;
            if (kizi::write(reinterpret_cast<uintptr_t>(lift), &val, 4)) {
                if (want < 1.0f && applied == 1.0f)
                    g_aero_lowered++;
                if (want < g_aero_min)
                    g_aero_min = want;
                applied = want;
            }
            continue;
        }
        live::Car car{static_cast<uint32_t>(i)};
        if (applied == 1.0f) {                     // start of a slide: remember the car's own value
            auto b = car.get(live::Field::aero_lift_scale);
            if (!b) {
                static bool told = false;
                if (!told)
                    told = true, swofty::log("[ai-tweaks] aero loss: car %d, reading aero_lift_scale failed (will retry)", i);
                continue;
            }
            g_aero_base[i] = *b;
        }
        float val = want == 1.0f ? g_aero_base[i] : g_aero_base[i] * want;
        swofty::Result r = car.set(live::Field::aero_lift_scale, val);
        if (r == swofty::Result::ok) {
            if (want < 1.0f && applied == 1.0f)
                g_aero_lowered++;
            if (want < g_aero_min)
                g_aero_min = want;
            applied = want;
        } else if (int(r) != g_aero_last_result) {
            g_aero_last_result = int(r);
            g_aero_refused++;
            swofty::log("[ai-tweaks] aero loss: car %d, setting aero_lift_scale to %.3f was refused (%s)", i, val, swofty::result_name(r));
        }
        if (r == swofty::Result::unavailable || r == swofty::Result::invalid) {
            swofty::log("[ai-tweaks] aero loss: the core refused aero_lift_scale (%s) - not used", swofty::result_name(r));
            g_aero_disabled = true;
            return;
        }
    }
}

// --- Terminal Damage and equal damage ------------------------------------------------------------------------------------
// Terminal Damage (nick.terminal-damage-probe) has its own suspension model ("Suspension damage from hits", and "... for AI
// cars") that replaces the game's suspension damage. While that model runs for a car, this addon writes no suspension damage
// of its own to it (no rear break, no jump damage, no suspension factors for AI cars).
// 1.2.1: only when the model is really on. Until 1.2.0 Terminal Damage's DLL being loaded was enough, so with it installed and
// its suspension damage off (its default) no rear corner broke at all, and no wheel-over-wheel launch either (1.2.0 test).
// Its two options are read once at load from Swofty's settings.json (they need a game restart anyway).
bool g_td_susp = false, g_td_susp_ai = false;      // Terminal Damage's suspension model: your car / AI cars
bool td_model(uint64_t veh)                        // Terminal Damage's suspension model runs for this car (veh 0 = an AI car)
{
    return (veh && human(veh) ? g_td_susp : g_td_susp_ai) && GetModuleHandleA("terminal_damage_probe.dll") != nullptr;
}
bool td_owns_suspension(uint64_t veh) { return td_defer.get() && td_model(veh); }   // 1.2.2: never (td_defer off)
std::atomic<DWORD> g_own_break[32];                // when this addon broke a corner of car i (break watcher, below)

// "current" state index of Terminal Damage option `key` in settings.json text `s` (after its addon id at `from`), or -1
int td_option(const std::string &s, size_t from, const char *key)
{
    size_t k = s.find(std::string("\"") + key + "\"", from);
    if (k == std::string::npos)
        return -1;
    size_t c = s.find("\"current\"", k);
    if (c == std::string::npos || c - k > 1500)    // must belong to this option
        return -1;
    for (c += 9; c < s.size() && (s[c] == ' ' || s[c] == ':' || s[c] == '\t' || s[c] == '\r' || s[c] == '\n'); c++)
        ;
    return c < s.size() && s[c] >= '0' && s[c] <= '9' ? s[c] - '0' : -1;
}
void read_td_settings()
{
    char dll[MAX_PATH] = {}, exe[MAX_PATH] = {};
    HMODULE self = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCSTR>(&read_td_settings), &self);
    GetModuleFileNameA(self, dll, MAX_PATH);
    GetModuleFileNameA(nullptr, exe, MAX_PATH);
    std::string a(dll), b(exe), text, from;
    for (int up = 0; up < 4 && a.rfind('\\') != std::string::npos; up++)   // ...\swofty\addons\<id>\native\<dll> -> ...\swofty
        a.resize(a.rfind('\\'));
    if (b.rfind('\\') != std::string::npos)
        b.resize(b.rfind('\\'));
    for (const std::string &path : {a + "\\settings.json", b + "\\swofty\\settings.json"}) {
        if (FILE *f = fopen(path.c_str(), "rb")) {
            char buf[16384];
            for (size_t n; (n = fread(buf, 1, sizeof buf, f)) > 0;)
                text.append(buf, n);
            fclose(f);
        }
        if (!text.empty()) {
            from = path;
            break;
        }
    }
    size_t td = text.find("\"nick.terminal-damage-probe\"");
    if (td == std::string::npos) {
        swofty::log("[ai-tweaks] Terminal Damage: %s - rear breaks are done here", text.empty() ? "Swofty's settings.json not readable"
                                                                                            : "not in this profile");
        return;
    }
    g_td_susp = td_option(text, td, "suspension_damage") == 1;
    g_td_susp_ai = g_td_susp && td_option(text, td, "suspension_damage_ai") == 1;   // needs the first one
    swofty::log("[ai-tweaks] Terminal Damage: suspension damage from hits %s, for AI cars %s (%s) - Contact Physics has priority: "
                "its own suspension damage runs for every car%s", g_td_susp ? "ON" : "off", g_td_susp_ai ? "ON" : "off",
                from.c_str(), g_td_susp || g_td_susp_ai ? " (Terminal Damage's model adds its own on top)" : "");
}

// The game applies its damage setting with a different factor for your car and for AI cars (research/DAMAGE_RE.md,
// 0x142f182e0, vehicle flag bit 3): wing component +0x40/+0x44, suspension component +0x128/+0x12c, each body part +0x50/+0x54.
// "Equal damage" copies your car's factors onto every AI car (re-checked every 2 s, the game sets them again per session),
// so an AI car takes as much damage from a hit as your car would.
// Front suspension toughness (0.9.12). The game's own suspension damage breaks front corners about as easily as rear ones
// (seen in test: your car lost a front corner twice in one race, all from the game's damage). In F1 the rear is the
// vulnerable end. The F1SuspensionDamage component ([veh+0x5d0]) keeps its damage scales XOR-scrambled: front = (+0x38) ^
// (+0x40), rear = (+0x48) ^ (+0x50) as floats (research/DAMAGE_RE.md). "Tougher" halves the front scale, "Much tougher"
// quarters it (every car, yours too); the rear is left to the game. Checked every 2 s: a component whose front scale is not
// the one we wrote (new car, or the game set it again for a new session) is scaled again.
void front_tough_tick()
{
    std::string_view v = front_tough.get();
    float f = v == "tougher" ? 0.5f : v == "much" ? 0.25f : 1.0f;
    static uint64_t done[64];
    static float wrote[64];
    static int next = 0, logged = 0;
    if (f >= 1.0f || !g_vehicles)
        return;
    for (int i = 0; i < 32; i++) {
        uint64_t veh = 0, comp = 0;
        if (!kizi::read(g_vehicles + i * 8, veh) || !veh || !kizi::read(veh + 0x5d0, comp) || !comp || td_owns_suspension(veh))
            continue;
        int slot = -1;
        for (int k = 0; k < 64; k++)
            if (done[k] == comp)
                slot = k;
        uint32_t fa = 0, fb = 0, ra = 0, rb = 0;
        if (!kizi::read(comp + 0x38, fa) || !kizi::read(comp + 0x40, fb) || !kizi::read(comp + 0x48, ra) || !kizi::read(comp + 0x50, rb))
            continue;
        uint32_t fx = fa ^ fb, rx = ra ^ rb;
        float front, rear;
        memcpy(&front, &fx, 4), memcpy(&rear, &rx, 4);
        if (slot >= 0 && front == wrote[slot])
            continue;                              // already ours
        if (slot < 0)
            slot = next, next = (next + 1) & 63, done[slot] = comp;
        wrote[slot] = front;                       // until a good value is written: leave this one alone
        if (!std::isfinite(front) || front <= 0 || front > 1e6f)
            continue;                              // not the expected layout: leave it
        float want = front * f;
        uint32_t wx;
        memcpy(&wx, &want, 4);
        uint32_t nb = fa ^ wx;
        if (kizi::write(comp + 0x40, &nb, 4))
            wrote[slot] = want;
        if (logged < 3) {
            logged++;
            swofty::log("[ai-tweaks] front suspension: car %d front damage scale %g -> %g (rear %g, game)", i, front, want, rear);
        }
    }
}

int g_equal_logged = -1;
void equal_damage_tick()
{
    if (!g_vehicles || !equal_damage.get())
        return;
    uint64_t me = 0;
    for (int i = 0; i < 32 && !me; i++) {
        uint64_t v = 0;
        if (kizi::read(g_vehicles + i * 8, v) && v && human(v))
            me = v;
    }
    uint64_t my_wing = 0, my_susp = 0, my_body = 0, pb = 0, pe = 0;
    if (!me || !kizi::read(me + 0x5c8, my_wing) || !kizi::read(me + 0x5d0, my_susp) || !kizi::read(me + 0x5e0, my_body))
        return;
    float wing[2] = {}, susp[2] = {};
    bool have_wing = my_wing && kizi::read(my_wing + 0x40, wing, 8);
    bool have_susp = my_susp && kizi::read(my_susp + 0x128, susp, 8) && !td_owns_suspension(0);
    std::vector<std::array<float, 2>> parts;
    if (my_body && kizi::read(my_body + 0x58, pb) && kizi::read(my_body + 0x60, pe) && pb && pe > pb && (pe - pb) / 0x60 <= 64)
        for (uint64_t p = pb; p < pe; p += 0x60) {
            std::array<float, 2> f{};
            if (!kizi::read(p + 0x50, f.data(), 8))
                break;
            parts.push_back(f);
        }
    int changed = 0;
    for (int i = 0; i < 32; i++) {
        uint64_t v = 0, c = 0, b = 0, e = 0;
        if (!kizi::read(g_vehicles + i * 8, v) || !v || v == me || human(v))
            continue;
        bool any = false;
        auto put = [&](uint64_t at, const float *want) {
            float now[2] = {};
            if (kizi::read(at, now, 8) && (now[0] != want[0] || now[1] != want[1]) && kizi::write(at, want, 8))
                any = true;
        };
        if (have_wing && kizi::read(v + 0x5c8, c) && c)
            put(c + 0x40, wing);
        if (have_susp && kizi::read(v + 0x5d0, c) && c)
            put(c + 0x128, susp);
        if (!parts.empty() && kizi::read(v + 0x5e0, c) && c && kizi::read(c + 0x58, b) && kizi::read(c + 0x60, e) && b && e > b &&
            (e - b) / 0x60 == parts.size())    // same body part list as your car (same car model): copy part by part
            for (size_t k = 0; k < parts.size(); k++)
                put(b + k * 0x60 + 0x50, parts[k].data());
        changed += any;
    }
    if (changed && changed != g_equal_logged) {
        swofty::log("[ai-tweaks] equal damage: %d AI car(s) set to your car's damage factors (wing %.3g/%.3g, suspension %s, "
                    "%zu body parts)", changed, wing[0], wing[1], have_susp ? "yes" : "left to Terminal Damage", parts.size());
        g_equal_logged = changed;
    }
}

// --- suspension after a jump ---------------------------------------------------------------------------------------------
// A car that flies (launched over a wheel, a kerb, another car) can land with broken suspension. The worker watches every
// car's vertical speed (body +0x224) at 20 Hz: free fall (vertical acceleration about -g or less for 0.15 s) means the car is
// in the air; ground forces again mean it landed. After a flight of at least 0.3 s a chance roll decides whether one corner
// is damaged (longer flights and harder landings are more likely). The damage goes where the game keeps it
// (research/DAMAGE_RE.md): the F1SuspensionDamage component [vehicle+0x5d0], counters rear L/R +0x130/+0x134 and front L/R
// +0x138/+0x13c. The game treats a corner as broken (its own getters 0x144655530..0x144655590) once the counter reaches the
// threshold it keeps XOR-scrambled at +0x58 ^ +0x60; "Damage" puts the corner at 70-95 % of that (the car keeps going,
// damaged), "Break" at the threshold.
float jump_chance()
{
    std::string_view v = jump_break.get();
    return v == "rare" ? 0.2f : v == "sometimes" ? 0.45f : v == "often" ? 0.8f : 0.0f;
}

struct Flight {
    uint64_t veh = 0, body = 0;                    // body cached per car (stable for the session): 2 reads per car per tick
    float vy = 0, vy_min = 0, vy_max = 0;
    DWORD last = 0, start = 0, rolled = 0;
    int fall = 0, ground = 0;
    bool air = false;
    DWORD landed_t = 0;
};
Flight g_flights[32];
uint32_t g_jump_rng = 0x9e3779b9u;
float jump_rnd()
{
    g_jump_rng ^= g_jump_rng << 13, g_jump_rng ^= g_jump_rng >> 17, g_jump_rng ^= g_jump_rng << 5;
    return float(g_jump_rng & 0xFFFFFF) / float(0x1000000);
}

float susp_threshold(uint64_t comp)
{
    uint32_t a = 0, b = 0;
    if (!kizi::read(comp + 0x58, a) || !kizi::read(comp + 0x60, b))
        return 0;
    uint32_t x = a ^ b;
    float f;
    memcpy(&f, &x, 4);
    return std::isfinite(f) && f > 0 && f < 1e6f ? f : 0;
}

void landed(int idx, Flight &f, float secs)
{
    float chance = jump_chance();
    uint64_t comp = 0;
    bool you = human(f.veh);
    float landing = std::max(0.0f, -f.vy_min);                       // vertical speed at touchdown (m/s)
    // 0.6.5: a real jump goes up and comes down. Cars going over a crest (vertical speed falling towards 0, never really
    // falling) and a car bouncing on another one every 2 s were counted in 0.6.4; now the car must land at 1.5 m/s or more
    // after having risen, and each car rolls at most once per 10 s.
    if (td_owns_suspension(f.veh)) {               // Terminal Damage runs its own suspension model
        static DWORD told = 0;
        if (GetTickCount() - told > 60000)
            swofty::log("[ai-tweaks] jump: car %d flew %.2f s - suspension left to Terminal Damage", idx, secs), told = GetTickCount();
        return;
    }
    if (chance <= 0 || secs < 0.3f || landing < 1.5f || f.vy_max < 0.5f || GetTickCount() - f.rolled < 10000 ||
        (you && !jump_you.get()) || !kizi::read(f.veh + 0x5d0, comp) || !comp)
        return;
    f.rolled = GetTickCount();
    float p = chance * std::clamp((secs - 0.3f) / 0.7f + std::max(0.0f, landing - 3.0f) / 6.0f, 0.15f, 1.0f);
    float roll = jump_rnd();
    if (roll >= p) {
        swofty::log("[ai-tweaks] jump: car %d (%s) flew %.2f s, landed at %.1f m/s, suspension held (roll %.2f >= %.2f)", idx,
                    you ? "YOU" : "AI", secs, landing, roll, p);
        return;
    }
    float thr = susp_threshold(comp);
    if (thr <= 0)
        return;
    static const char *const names[4] = {"rear left", "rear right", "front left", "front right"};
    int corner = int(jump_rnd() * 4) & 3;
    uintptr_t at = comp + 0x130 + 4 * corner;
    float now = 0;
    kizi::read(at, now);
    bool brk = jump_damage.get() == "break";
    if (!brk && now >= thr * 0.7f) {               // that corner is already badly damaged: leave it
        swofty::log("[ai-tweaks] jump: car %d (%s) flew %.2f s, %s suspension already damaged (%.2f of %.2f)", idx,
                    you ? "YOU" : "AI", secs, names[corner], now, thr);
        return;
    }
    float to = brk ? thr * 1.02f : std::min(thr * 0.95f, std::max(now, thr * (0.7f + 0.25f * jump_rnd())));
    if (to > now && kizi::write(at, &to, 4) && idx >= 0 && idx < 32)
        g_own_break[idx] = GetTickCount() | 1;
    swofty::log("[ai-tweaks] jump: car %d (%s) flew %.2f s, landed at %.1f m/s -> %s suspension %s (%.2f -> %.2f of %.2f)", idx,
                you ? "YOU" : "AI", secs, landing, names[corner], brk ? "BROKEN" : "damaged", now, std::max(now, to), thr);
    feed("%s LANDED %.1f M/S - %s %s", car_tag(f.veh).s, landing, names[corner], brk ? "BROKEN" : "DAMAGED");
}

// --- rear suspension from a hit (0.9.3) -----------------------------------------------------------------------------------
// In real F1 a front wing or front tyre into the rear tyre of the car ahead often breaks that car's rear suspension: the
// rear corner takes the hit from behind and from the side, the front one rarely does. On the first record of a touch where
// the contact point is at one car's REAR tyre (within 0.5 m of RL / RR, [[veh+0x580]+0x70 / +0x78]+0x2f0) and in FRONT of the
// other car's centre (its nose, front wing or front tyre), a chance roll breaks that rear corner: counter rear L / R +0x130 /
// +0x134 of [veh+0x5d0] set to the game's own breaking point (as "Break" for jumps). The chance grows with the speed of the
// car that is hit (none under 100 km/h, full from 250 km/h) and with how hard the hit was (closing speed: none under 1.5 m/s,
// full from 5 m/s, 0.9.4).
// At most once per car per 5 s. Left to Terminal Damage when it runs its own suspension model.
float rear_break_chance()
{
    std::string_view v = rear_break.get();
    return v == "rare" ? 0.3f : v == "sometimes" ? 0.6f : v == "often" ? 1.0f : 0.0f;   // 0.9.6: much harsher
}
std::atomic<int> g_rear_breaks{0}, g_rear_rolls{0};

// Break trace (0.9.7): after a break the worker samples that car every 0.25 s for 4 s (speed, yaw rate, how far it moved
// sideways) and logs one line, so the log shows whether it slid / spun on or stopped dead.
struct BreakTrace {
    std::atomic<uint64_t> veh{0};
    DWORD t0 = 0;
    int idx = -1, n = 0;
    char line[600] = {};
};
BreakTrace g_trace[4];

void start_trace(uint64_t veh, int idx)
{
    for (auto &t : g_trace)
        if (!t.veh.load()) {
            t.t0 = GetTickCount(), t.idx = idx, t.n = 0, t.line[0] = 0;
            t.veh = veh;
            return;
        }
}

void tick_traces()
{
    for (auto &t : g_trace) {
        uint64_t veh = t.veh.load();
        if (!veh)
            continue;
        DWORD el = GetTickCount() - t.t0;
        if (el < DWORD(t.n) * 250)
            continue;
        uint64_t b = body_of(veh);
        float v[3] = {}, w[3] = {};
        if (b && kizi::read(b + 0x220, v, 12) && kizi::read(b + 0x210, w, 12)) {
            size_t l = strlen(t.line);
            snprintf(t.line + l, sizeof t.line - l, "%s%.0f km/h %.0f deg/s", t.n ? " | " : "", len({v[0], v[1], v[2]}) * 3.6f,
                     std::fabs(w[1]) * 57.3f);
        }
        if (++t.n > 16 || !b) {
            swofty::log("[ai-tweaks] after the break, car %d every 0.25 s: %s", t.idx, t.line);
            t.veh = 0;
        }
    }
}

// --- kick when a suspension breaks (0.9.8) --------------------------------------------------------------------------------
// A suspension that fails at speed is violent: the corner drops or digs in and the car snaps sideways. Setting the corner's
// counter only makes the car sag. So at the moment a rear corner breaks the car also gets one kick from the game's own
// apply-impulse function, AT THAT CORNER: up (the corner pops, the car hops) and sideways along the car's own left axis, away
// from the car that hit it. Because it lands at the rear corner the sideways part also yaws the car round. Sizes (m/s of the
// whole car, up / sideways): Small 1.0 / 1.7, Medium 1.8 / 3.1, Big 3.0 / 5.1, scaled by the car's speed (30 % at 60 km/h, full
// from 200) and the hit (half under 1 m/s closing, full from 2 m/s). A hard tyre hit (from 1.5 m/s) that does not break the
// corner gets 40 % of it. Runs on the physics thread inside the contact handler, like the wheel lift.
std::atomic<int> g_kicks{0}, g_launches{0}, g_wing_on_tyre{0};
std::atomic<float> g_kick_max{0}, g_launch_max{0};

// Kick size (m/s of the whole car, upward at full size; sideways is x1.7): Small 1.0, Medium 1.8, Big 3.0, Huge 6.0.
float kick_up() { return crash().kick_up; }     // 1.1.0: from the Crashes level (0 / 1.8 / 3.0)
// Size at speed: 40 % at 100 km/h or less, full from 160 km/h (1.0: 30 % up to full at 200, so a 120 km/h break kicked 1 m/s)
float crash_scale(float kmh, float impact)
{
    return std::clamp((kmh - 60.0f) / 100.0f, 0.4f, 1.0f) * std::clamp(impact / 2.0f, 0.5f, 1.0f);
}

// Wheel over wheel (1.0.0): the chasing car's front tyre rides up the rear tyre ahead and the car is thrown up at that
// wheel. Up = 1.5 x the kick size (Big: 4.5 m/s), scaled like the kick, at the striker's front wheel, so its nose lifts.
void launch(uint64_t car, const float at[3], float kmh, float impact)
{
    float up = kick_up() * 1.5f;
    uint64_t b = body_of(car);
    float inv_mass = 0;
    if (up <= 0 || !g_apply_impulse || !b || !kizi::read(b + 0x438, inv_mass) || inv_mass <= 1e-6f)
        return;
    float s = crash_scale(kmh, impact);
    alignas(16) float J[4] = {0, up * s / inv_mass, 0, 0};
    alignas(16) float P[4] = {at[0], at[1], at[2], 1};
    g_apply_impulse(reinterpret_cast<void *>(b), J, P);
    lift_noted(b, up * s);
    g_launches++;
    g_pstats[preset_index()].launches++;
    if (up * s > g_launch_max)
        g_launch_max = up * s;
    feed("%s LAUNCHED %.1f M/S UP", car_tag(car).s, up * s);
}
float break_kick_at(uint64_t victim, uint64_t striker, const float at[3], float kmh, float impact, float share)
{                                                  // returns the sideways kick given (m/s), 0 = none
    float up = kick_up();
    uint64_t bv = body_of(victim), bs = body_of(striker);
    float pv[3], ps[3], left[3], inv_mass = 0;
    if (up <= 0 || !g_apply_impulse || !bv || !bs || !kizi::read(bv + 0x1c0, pv, 12) || !kizi::read(bs + 0x1c0, ps, 12) ||
        !kizi::read(bv + 0x190, left, 12) || !kizi::read(bv + 0x438, inv_mass) || inv_mass <= 1e-6f)
        return 0.0f;
    float lh = std::sqrt(left[0] * left[0] + left[2] * left[2]);
    if (lh < 0.3f)
        return 0.0f;                               // car on its side / upside down: no kick
    float lx = left[0] / lh, lz = left[2] / lh;
    float sign = (pv[0] - ps[0]) * lx + (pv[2] - ps[2]) * lz >= 0 ? 1.0f : -1.0f;   // away from the car that hit it
    float s = share * crash_scale(kmh, impact);
    float side = up * 1.7f * s, m = 1.0f / inv_mass;
    alignas(16) float J[4] = {sign * side * lx * m, up * s * m, sign * side * lz * m, 0};
    alignas(16) float P[4] = {at[0], at[1], at[2], 1};
    g_apply_impulse(reinterpret_cast<void *>(bv), J, P);
    lift_noted(bv, up * s);
    g_kicks++;
    g_pstats[preset_index()].kicks++;
    if (side > g_kick_max)
        g_kick_max = side;
    return side;
}

// Snap (1.1.0). A rear corner that fails at speed drags on the road and the car pivots round it: in the 1.0 traces a
// broken car only slowed down in a straight line (the game keeps its broken corner level). So after a rear break above
// 70 km/h the car's yaw rate is pushed to the level's snap rate (1.2.0: half at 110 km/h or less, full from 170) in the direction
// a drag at that corner turns it (torque of a force against the car's travel, applied at the corner), held 0.5 s and faded
// out by 0.9 s, by the same 1 ms hold as Classic's spin (it writes the body's yaw rate +0x214; a write in between physics
// steps sticks). Called inside on_record, which already holds g_spin_mx.
std::atomic<int> g_snap_count{0};
float snap(uint64_t victim, const float corner[3], float kmh, int idx)
{                                                  // returns the snap's yaw rate (deg/s), 0 = none
    float dps = crash().snap_dps;
    uint64_t b = body_of(victim);
    float pos[3], vel[3], up[3];
    if (dps <= 0 || kmh < 70.0f || !b || !kizi::read(b + 0x1c0, pos, 12) || !kizi::read(b + 0x220, vel, 12) ||
        !kizi::read(b + 0x1a0, up, 12))
        return 0.0f;
    V3 r{corner[0] - pos[0], corner[1] - pos[1], corner[2] - pos[2]}, v{vel[0], vel[1], vel[2]};
    float vl = len(v);
    if (vl < 1.0f)
        return 0.0f;
    V3 F{-v.x / vl, -v.y / vl, -v.z / vl};
    V3 tq{r.y * F.z - r.z * F.y, r.z * F.x - r.x * F.z, r.x * F.y - r.y * F.x};
    float sign = tq.x * up[0] + tq.y * up[1] + tq.z * up[2] >= 0 ? 1.0f : -1.0f;
    float peak = sign * dps / 57.2958f * std::clamp((kmh - 50.0f) / 120.0f, 0.5f, 1.0f);
    g_snaps.erase(std::remove_if(g_snaps.begin(), g_snaps.end(), [&](const Snap &s) { return s.body == b; }), g_snaps.end());
    g_snaps.push_back({b, peak, GetTickCount()});
    g_snap_count++;
    g_pstats[preset_index()].snaps++;
    swofty::log("[ai-tweaks] snap: car %d, rear corner broke at %.0f km/h -> yaw %.0f deg/s (crashes %.*s)", idx, kmh,
                std::fabs(peak) * 57.2958f, int(level_name().size()), level_name().data());
    return std::fabs(peak) * 57.2958f;
}

void rear_hit(uint64_t victim, uint64_t striker, const float pt[3], float impact)
{
    float base = rear_break_chance();
    uint64_t set = 0, comp = 0, bv = body_of(victim), bs = body_of(striker);
    float v[3], spos[3], sfwd[3];
    if (!bv || !bs || !kizi::read(victim + 0x580, set) || !set || !kizi::read(bv + 0x220, v, 12) ||
        !kizi::read(bs + 0x1c0, spos, 12) || !kizi::read(bs + 0x1b0, sfwd, 12))
        return;
    int corner = -1;
    float best = 0.7f;                             // 0.9.6: anywhere on the tyre (was 0.5 m from the wheel centre)
    float wpos[3] = {};
    for (int i = 0; i < 4; i++) {                  // 0 rear left, 1 rear right, 2 front left, 3 front right
        uint64_t w = 0;
        float wp[3];
        if (!kizi::read(set + 0x70 + 8 * i, w) || !w || !kizi::read(w + 0x2f0, wp, 12))
            continue;
        float d = len({wp[0] - pt[0], wp[1] - pt[1], wp[2] - pt[2]});
        if (d < best)
            best = d, corner = i, wpos[0] = wp[0], wpos[1] = wp[1], wpos[2] = wp[2];
    }
    V3 f{sfwd[0], sfwd[1], sfwd[2]};
    float fl = len(f), ahead = fl > 0.5f ? dot({pt[0] - spos[0], pt[1] - spos[1], pt[2] - spos[2]}, f) / fl : 0.0f;
    if (corner < 0 || corner > 1 || ahead < 0.0f)
        return;                                    // not at a REAR tyre (0.9.7: front corners are the game's), or hit from behind
    // 1.0.0: only the other car's FRONT TYRE counts (wheel on wheel). A front wing into a rear tyre breaks nothing here (the
    // wing takes the damage); a front tyre on the rear tyre launches the car behind (below) and can break the corner.
    uint64_t sset = 0;
    float swheel[3] = {};
    bool tyre = false;
    if (kizi::read(striker + 0x580, sset) && sset)
        for (int i = 2; i < 4 && !tyre; i++) {     // FL / FR
            uint64_t w = 0;
            if (kizi::read(sset + 0x70 + 8 * i, w) && w && kizi::read(w + 0x2f0, swheel, 12) &&
                len({swheel[0] - pt[0], swheel[1] - pt[1], swheel[2] - pt[2]}) < 0.6f)
                tyre = true;
        }
    if (!tyre) {
        g_wing_on_tyre++;
        return;
    }
    float kmh = len({v[0], v[1], v[2]}) * 3.6f;
    if (impact >= 1.0f && (!human(striker) || your_car.get()))
        launch(striker, swheel, kmh, impact);      // wheel over wheel: the car behind climbs and flies
    // 1.2.1: the launch is contact physics and always runs; the break needs the setting, and not Terminal Damage's model
    if (base <= 0 || td_owns_suspension(victim) || !kizi::read(victim + 0x5d0, comp) || !comp)
        return;
    // 0.9.6 (experiment, "every collision should cost"): from 60 km/h (full at 160) and from 0.3 m/s closing (full at 2 m/s);
    float p = base * std::clamp((kmh - 60.0f) / 100.0f, 0.0f, 1.0f) * std::clamp((impact - 0.3f) / 1.7f, 0.0f, 1.0f);
    if (p <= 0)
        return;
    static uint64_t last_car[32];
    static DWORD last_t[32];
    int slot = int((victim >> 4) & 31);
    if (last_car[slot] == victim && GetTickCount() - last_t[slot] < 3000)
        return;
    last_car[slot] = victim, last_t[slot] = GetTickCount();
    g_rear_rolls++;
    float roll = jump_rnd();
    float thr = susp_threshold(comp);
    if (roll >= p || thr <= 0) {
        if (impact >= 1.5f)
            break_kick_at(victim, striker, wpos, kmh, impact, 0.4f);   // a hard hit that did not break it still jolts the car
        return;
    }
    uintptr_t at = comp + 0x130 + 4 * corner;
    float now = 0, to = thr * 1.02f;
    if (kizi::read(at, now) && to > now && kizi::write(at, &to, 4)) {
        if (int id = car_id(victim); id >= 0)
            g_own_break[id] = GetTickCount() | 1;
        g_rear_breaks++;
        g_pstats[preset_index()].breaks++;
        float side = break_kick_at(victim, striker, wpos, kmh, impact, 1.0f);
        float yaw = snap(victim, wpos, kmh, car_id(victim));
        if (realistic())
            arm_slide(victim, bv, 4000);           // 1.2.0: the game locks a broken car to a stop - let it slide on
        if (!human(victim))
            broken(car_id(victim));                // AI Driver Behaviour: how the AI drives a broken car ("retiring AI cars")
        start_trace(victim, car_id(victim));
        static const char *const names[4] = {"rear left", "rear right", "front left", "front right"};
        swofty::log("[ai-tweaks] hit: car %d (%s) hit at %.0f km/h (closing %.1f m/s) -> %s suspension BROKEN (chance %.2f), kick "
                    "%.1f m/s sideways", car_id(victim), human(victim) ? "YOU" : "AI", kmh, impact, names[corner], p, side);
        feed("%s %s BROKEN %.0f KM/H", car_tag(victim).s, names[corner], kmh);
        if (side > 0 || yaw > 0)
            feed("  KICK %.1f M/S  SNAP %.0f DEG/S", side, yaw);
    }
}

// --- rear breaks by the game or Terminal Damage (1.2.1) --------------------------------------------------------------------
// A rear corner also breaks without this addon: a wall hit (the game's own suspension damage) or Terminal Damage's model. The
// worker checks every car's rear counters every 50 ms; a corner that reaches its breaking point at 70 km/h or more and was not
// broken by this addon in the last 1.5 s gets the same crash as ours minus the kick (there is no car to kick it away from):
// the snap, the slide, and AI Driver Behaviour is told. Rigid body only (the slide is part of it).
std::atomic<int> g_ext_breaks{0};
void watch_breaks()
{
    if (!g_vehicles || !realistic())
        return;
    static uint64_t seen[32];
    static uint8_t broke[32];
    static const char *const names[2] = {"rear left", "rear right"};
    for (int i = 0; i < 32; i++) {
        uint64_t veh = 0, comp = 0;
        float c[2] = {}, thr = 0;
        if (!kizi::read(g_vehicles + i * 8, veh) || !veh || !kizi::read(veh + 0x5d0, comp) || !comp ||
            (thr = susp_threshold(comp)) <= 0 || !kizi::read(comp + 0x130, c, 8)) {
            seen[i] = 0;
            continue;
        }
        uint8_t now = uint8_t((c[0] >= thr ? 1 : 0) | (c[1] >= thr ? 2 : 0));
        if (seen[i] != veh) {                      // a new car (new session): take its corners as they are
            seen[i] = veh, broke[i] = now;
            continue;
        }
        uint8_t fresh = uint8_t(now & ~broke[i]);
        broke[i] = now;
        DWORD own = g_own_break[i].load();
        bool you = fresh && human(veh);
        if (!fresh || (own && GetTickCount() - own < 1500) || (you && !your_car.get()))
            continue;
        int corner = fresh & 1 ? 0 : 1;
        uint64_t body = body_of(veh), set = 0, w = 0;
        float v[3], wp[3];
        if (!body || !kizi::read(body + 0x220, v, 12) || !kizi::read(veh + 0x580, set) || !set ||
            !kizi::read(set + 0x70 + 8 * corner, w) || !w || !kizi::read(w + 0x2f0, wp, 12))
            continue;
        float kmh = len({v[0], v[1], v[2]}) * 3.6f, yaw = 0;
        if (kmh < 70.0f)
            continue;
        {
            std::lock_guard<std::mutex> lock(g_spin_mx);
            yaw = snap(veh, wp, kmh, i);
            arm_slide(veh, body, 4000);
        }
        if (!you)
            broken(i);
        start_trace(veh, i);
        g_ext_breaks++;
        bool td = td_model(veh);
        swofty::log("[ai-tweaks] rear break by %s: car %d (%s) %s at %.0f km/h -> snap %.0f deg/s, slides on (crashes %.*s)",
                    td ? "Terminal Damage" : "the game", i, you ? "YOU" : "AI", names[corner], kmh, yaw,
                    int(level_name().size()), level_name().data());
        feed("%s %s BROKE %.0f KM/H  SNAP %.0f DEG/S", car_tag(veh).s, names[corner], kmh, yaw);
    }
}

// --- loose wheels (0.9.4) ------------------------------------------------------------------------------------------------
// A wheel torn off a car hangs on the game's tethers: it stays joined to the car and drags it, so a car whose suspension
// breaks at 250 km/h stops dead as if something held it. "Loose wheels" lets such a wheel fly off as free debris instead.
// Same method as Terminal Damage's "Loose AI wheels" (nick.terminal-damage-probe, MIT): around the game's wheel-detach
// function (vtable slot +0xa0 of the wheel part) the part's tether node (+0xe0) is set to -1, so the game takes its own
// "no tether node" path while it creates the free wheel; the node is put back right after. Only for a part that is really
// coming off: its vtable is the wheel part's, car (+0x10) set, no body (+0x40) or connection (+0x100) yet, not active (+0x10c).
// Terminal Damage checks two addresses of the old game build and turns this off on the November 2026 build; this version
// finds the detach function by its bytes and the wheel part vtable by searching the image for a pointer to it at slot +0xa0.
// If the function's first bytes are not the expected ones (another addon already hooks it) nothing is installed.
using DetachFn = uint64_t (*)(void *, uint64_t);
DetachFn g_detach_orig;
uintptr_t g_wheel_vt;
std::atomic<int> g_loose{0};

uint64_t detach_hook(void *self, uint64_t notify)
{
    auto part = static_cast<uint8_t *>(self);
    std::string_view mode = loose_wheels.get();
    int32_t *node = reinterpret_cast<int32_t *>(part + 0xe0);
    uint64_t car = *reinterpret_cast<uint64_t *>(part + 0x10);
    bool loose = mode != "off" && *reinterpret_cast<uintptr_t *>(part) == g_wheel_vt && car &&
                 !*reinterpret_cast<uint64_t *>(part + 0x40) && !*reinterpret_cast<uint64_t *>(part + 0x100) && !part[0x10c] &&
                 *node >= 0 && (mode == "all" || !human(car));
    // 0.9.7: once per wheel part. In 0.9.4 - 0.9.6 the game called detach on the same part again and again (up to 31,000
    // times in 30 s after a pile-up) and every call took the no-tether path; now a part seen in the last 60 s is left stock.
    // 0.9.10: 10 minutes (0.9.9 logs: the game calls detach again on the same 8 parts every minute, so they were redone).
    if (loose) {
        static uintptr_t seen[64];
        static DWORD seen_t[64];
        static int next = 0;
        DWORD now = GetTickCount();
        for (int i = 0; i < 64 && loose; i++)
            if (seen[i] == uintptr_t(self) && now - seen_t[i] < 600000)
                loose = false;
        if (loose)
            seen[next] = uintptr_t(self), seen_t[next] = now, next = (next + 1) & 63;
    }
    int32_t keep = *node;
    if (loose)
        *node = -1;                                // the game's own "no tether node" path while the free wheel is made
    uint64_t r = g_detach_orig(self, notify);
    if (loose) {
        *node = keep;
        g_loose++;
    }
    return r;
}

const char *const DETACH =
    "41 57 41 56 41 55 41 54 56 57 55 53 48 81 EC 38 03 00 00 C5 78 29 BC 24 20 03 00 00 C5 78 29 B4 24 10 03 00 00 C5 78 29 AC "
    "24 00 03 00 00 C5 78 29 A4 24 F0 02 00 00 C5 78 29 9C 24 E0 02 00 00 C5 78 29 94 24 D0 02 00 00 C5 78 29 8C 24 C0 02 00 00 "
    "C5 78 29 84 24 B0 02 00 00 C5 F8 29 BC 24 A0 02 00 00 C5 F8 29 B4 24 90 02 00 00 48 83 79 40 00 74 6E";

void install_loose_wheels()
{
    uintptr_t at = kizi::scan_code(DETACH, "ai-tweaks: wheel detach");
    if (!at)
        return;
    // the wheel part vtable: a pointer to the detach function at its slot +0xa0, somewhere in the image's data
    auto base = kizi::image_base();
    auto dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(base);
    auto nt = reinterpret_cast<const IMAGE_NT_HEADERS *>(base + dos->e_lfanew);
    auto sec = IMAGE_FIRST_SECTION(nt);
    int found = 0;
    for (int i = 0; i < nt->FileHeader.NumberOfSections && found < 2; i++) {
        if ((sec[i].Characteristics & IMAGE_SCN_MEM_EXECUTE) || !(sec[i].Characteristics & IMAGE_SCN_MEM_READ))
            continue;
        uintptr_t s0 = base + sec[i].VirtualAddress, n = sec[i].Misc.VirtualSize & ~uintptr_t(7);
        std::vector<uint64_t> buf(1 << 16);
        for (uintptr_t off = 0; off < n && found < 2; off += buf.size() * 8) {
            size_t len = std::min<uintptr_t>(buf.size() * 8, n - off);
            if (!kizi::read(s0 + off, buf.data(), len))
                continue;
            for (size_t k = 0; k < len / 8; k++)
                if (buf[k] == at) {
                    uintptr_t vt = s0 + off + k * 8 - 0xa0;
                    uint64_t first = 0;
                    if (kizi::read(vt, first) && kizi::in_image(first))
                        g_wheel_vt = found++ ? 0 : vt;   // must be exactly one
                }
        }
    }
    if (!g_wheel_vt || found != 1) {
        swofty::log("[ai-tweaks] loose wheels: wheel part vtable not found (%d candidate(s)) - not used", found);
        return;
    }
    static const uint8_t expect[19] = {0x41, 0x57, 0x41, 0x56, 0x41, 0x55, 0x41, 0x54, 0x56, 0x57,
                                       0x55, 0x53, 0x48, 0x81, 0xEC, 0x38, 0x03, 0x00, 0x00};
    uint8_t now[19] = {};
    if (!kizi::read(at, now, 19) || memcmp(now, expect, 19)) {
        swofty::log("[ai-tweaks] loose wheels: the detach function is already hooked (another addon) - not used");
        return;
    }
    uint8_t *tramp = kizi::alloc_near(at, 64);     // the 19 moved bytes (no relative addressing), then jmp back
    if (!tramp)
        return;
    memcpy(tramp, expect, 19);
    const uint8_t jmp[6] = {0xFF, 0x25, 0, 0, 0, 0};   // jmp qword [rip+0]
    uint64_t back = at + 19, hook = reinterpret_cast<uint64_t>(&detach_hook);
    memcpy(tramp + 19, jmp, 6), memcpy(tramp + 25, &back, 8);
    FlushInstructionCache(GetCurrentProcess(), tramp, 33);
    g_detach_orig = reinterpret_cast<DetachFn>(tramp);
    uint8_t patch[19];
    memcpy(patch, jmp, 6), memcpy(patch + 6, &hook, 8);
    memset(patch + 14, 0x90, 5);
    bool ok = kizi::patch_code(at, patch, 19);
    swofty::log("[ai-tweaks] loose wheels %s (detach %llx, wheel vtable rva %llx, %.*s)", ok ? "ready" : "NOT installed",
                (unsigned long long)at, (unsigned long long)(g_wheel_vt - base), int(loose_wheels.get().size()),
                loose_wheels.get().data());
}

void track_jumps()
{
    if (!g_vehicles || jump_chance() <= 0)
        return;
    DWORD t = GetTickCount();
    float fall_a = -std::min(7.0f, crash().fly_g - 1.0f);   // 1.1.1: a held flight falls at the level's gravity
    for (int i = 0; i < 32; i++) {
        Flight &f = g_flights[i];
        uint64_t veh = 0;
        float vy = 0;
        if (!kizi::read(g_vehicles + i * 8, veh) || !veh) {
            f = Flight{};
            continue;
        }
        if (f.veh != veh) {                        // new car in this slot (new session)
            f = Flight{};
            f.veh = veh, f.body = body_of(veh);
        }
        if (!f.body || !kizi::read(f.body + 0x224, vy) || !std::isfinite(vy)) {
            f.body = body_of(veh);                 // the body moved (car rebuilt): find it again next tick
            continue;
        }
        if (!f.last) {
            f.vy = vy, f.last = t;
            continue;
        }
        float dt = float(t - f.last) / 1000.0f;
        if (dt < 0.03f)
            continue;
        if (dt > 0.2f) {                           // paused / hitch: start over
            f.vy = vy, f.last = t, f.fall = f.ground = 0, f.air = false;
            continue;
        }
        float a = (vy - f.vy) / dt;                // vertical acceleration (m/s^2); free fall is about -9.8 or less
        if (!f.air) {
            f.fall = a < fall_a ? f.fall + 1 : 0;
            f.vy_max = f.fall ? std::max(f.vy_max, f.vy) : f.vy;       // vertical speed when the fall began (did it rise?)
            if (f.fall >= 3)
                f.air = true, f.start = t - 150, f.vy_min = vy, f.ground = 0;
        } else {
            f.vy_min = std::min(f.vy_min, vy);
            f.ground = a > -3.0f ? f.ground + 1 : 0;
            if (f.ground >= 2) {
                // 0.9.14: 4 AI cars "flew" at once 2 s after your crash (the game's terminal damage slow motion / replay).
                // Three or more cars in the air or landed within 1 s = not jumps: no damage roll.
                int together = 0;
                for (int k = 0; k < 32; k++)
                    if (k != i && (g_flights[k].air || (g_flights[k].landed_t && t - g_flights[k].landed_t < 1000)))
                        together++;
                if (together >= 2) {
                    static DWORD told = 0;         // 1.2.1: once per burst (a restart or flashback drops every car at once)
                    if (t - told > 3000)
                        told = t, swofty::log("[ai-tweaks] jump: %d car(s) in the air at once - restart / replay / slow motion, "
                                              "ignored", together + 1);
                } else
                    landed(i, f, float(t - f.start - 100) / 1000.0f);
                f.landed_t = t;
                f.air = false, f.fall = 0;
            } else if (t - f.start > 6000)
                f.air = false, f.fall = 0;     // something else (a long fall off the world): give up
        }
        f.vy = vy, f.last = t;
    }
}

void preset_summary(int k)
{
    PresetStats &ps = g_pstats[k];
    int n = ps.touches.load(), lg = ps.logged.load();
    if (!n)
        return;
    std::string_view name = index_name(k);
    char extra[160] = "";
    if (lg)
        snprintf(extra, sizeof extra, " | %d touch(es) logged: average sideways peak %.0f km/h, biggest spin %.0f deg/s", lg,
                 double(ps.side_sum.load() / float(lg)), double(ps.spin_max.load()));
    swofty::log("[ai-tweaks] crashes summary %.*s: %d touch(es) (average closing %.1f m/s, hardest %.1f), %d break(s), %d snap(s), "
                "%d launch(es), %d kick(s), %d flight(s) (highest %.2f m), %d slide(s) (farthest %.0f m), %d blow-over(s) (steepest "
                "nose up %.0f deg)%s", int(name.size()), name.data(), n, double(ps.closing_sum.load() / float(n)),
                double(ps.closing_max.load()), ps.breaks.load(), ps.snaps.load(), ps.launches.load(), ps.kicks.load(),
                ps.flights.load(), double(ps.fly_max.load()), ps.slides.load(), double(ps.slide_max.load()), ps.blowovers.load(),
                double(ps.aoa_max.load()), extra);
}

// P key (0.9.15; 1.1.0: the Crashes level, Realistic -> Dramatic -> Hollywood -> Realistic). Only
// while the game window is in front, no Ctrl / Alt / Shift held, and a session with your car is loaded (a race, or its pause
// menu; not the main menus). Lasts until the game closes; the setting in Swofty stays as it is. 0.9.16: no beeps; a small
// label in the top-left corner for 2.5 s ("CONTACT  DRAMATIC  3/6"), drawn with Swofty's overlay in the 5x7 box font of
// osd.hpp (from Canned Events: the overlay's own text call drew nothing then).
std::atomic<DWORD> g_preset_shown{0};

// Priority notice (1.2.2). With Terminal Damage installed, a small box under the level label says for 7 s, once per game start
// (at the first session with your car), that Contact Physics has priority over its suspension damage, and whether Terminal
// Damage's own model is on (then both run: it adds its wall-hit damage, ours is never held back).
std::atomic<DWORD> g_notice_t{0};
std::atomic<const char *> g_notice_line{""};
void notice_tick()
{
    static bool done = false;
    if (done || !g_vehicles || !GetModuleHandleA("terminal_damage_probe.dll"))
        return;
    bool session = false;
    for (int i = 0; i < 32 && !session; i++) {
        uint64_t v = 0;
        session = kizi::read(g_vehicles + i * 8, v) && v && human(v) && body_of(v);
    }
    if (!session)
        return;
    done = true;
    bool on = g_td_susp || g_td_susp_ai;
    g_notice_line = on ? "OVER TERMINAL DAMAGE SUSPENSION" : "TERMINAL DAMAGE SUSPENSION IS OFF";
    g_notice_t = GetTickCount() | 1;
    swofty::log("[ai-tweaks] notice: Contact Physics has priority over Terminal Damage's suspension damage (its model %s)",
                on ? "is ON: both run, ours is never held back" : "is off");
}

void paint_preset(SwoftyPaint *p, void *)
{
    float w = 0, h = 0;
    swofty::overlay::size(p, &w, &h);
    if (w < 100 || h < 100)
        return;
    float px = h / 360 > 2 ? h / 360 : 2;          // 3 px at 1080p: small
    float pad = 3 * px, gap = 4 * px, bh = 7 * px + 2 * pad;
    float bx = w * 0.012f, by = h * 0.11f;
    DWORD now = GetTickCount(), t0 = g_preset_shown.load();
    if (DWORD n0 = g_notice_t.load(); n0 && now - n0 <= 7000) {   // 1.2.2: priority notice (notice_tick)
        std::string a = "CONTACT PHYSICS HAS PRIORITY", b = g_notice_line.load();
        float nw = float(std::max(a.size(), b.size())) * 6 * px + 2 * pad - px, nh = 16 * px + 2 * pad;
        swofty::overlay::rect(p, bx, by, nw, nh, 0x000000b4u);
        swofty::overlay::rect(p, bx, by, px, nh, 0xe10600ffu);   // red accent on the left
        osd::draw_text(p, bx + pad, by + pad, px, 0xffffffffu, a);
        osd::draw_text(p, bx + pad, by + pad + 9 * px, px, 0x9a9a9affu, b);
        by += nh + 2 * px;
    }
    int k = g_level_key.load();
    if (t0 && now - t0 <= 2500 && k >= 0 && k <= 2) {
        std::string name(CRASH_LEVELS[k]);
        for (char &c : name)
            c = char(c >= 'a' && c <= 'z' ? c - 32 : c);
        std::string a = "CRASHES", b = name + "  " + std::to_string(k + 1) + "/3";
        float wa = a.size() * 6 * px, wb = b.size() * 6 * px;
        float bw = wa + gap + wb + 2 * pad - px;
        swofty::overlay::rect(p, bx, by, bw, bh, 0x000000b4u);
        swofty::overlay::rect(p, bx, by, px, bh, 0xe10600ffu);   // red accent on the left
        osd::draw_text(p, bx + pad, by + pad, px, 0x9a9a9affu, a);
        osd::draw_text(p, bx + pad + wa + gap, by + pad, px, 0xffffffffu, b);
        by += bh + 2 * px;
    }
    if (!contact_log.get())
        return;
    FeedLine lines[4];                             // crash feed (testing), newest on top
    unsigned next;
    {
        std::lock_guard<std::mutex> lock(g_feed_mx);
        memcpy(lines, g_feed, sizeof lines);
        next = g_feed_next;
    }
    for (unsigned i = 0; i < 4; i++) {
        const FeedLine &l = lines[(next - 1 - i) & 3];
        if (!l.t || now - l.t > 6000)
            continue;
        std::string text(l.text);
        float bw = text.size() * 6 * px + 2 * pad - px;
        swofty::overlay::rect(p, bx, by, bw, bh, 0x000000b4u);
        swofty::overlay::rect(p, bx, by, px, bh, 0xffb000ffu);   // amber accent: a crash event
        osd::draw_text(p, bx + pad, by + pad, px, 0xffffffffu, text);
        by += bh + px;
    }
}
void preset_key()
{
    static bool was_down = false;
    bool down = (GetAsyncKeyState('P') & 0x8000) != 0;
    bool press = down && !was_down;
    was_down = down;
    if (!press || physics_mode.get() != "realistic" || (GetAsyncKeyState(VK_CONTROL) & 0x8000) ||
        (GetAsyncKeyState(VK_MENU) & 0x8000) || (GetAsyncKeyState(VK_SHIFT) & 0x8000))
        return;
    DWORD pid = 0;
    GetWindowThreadProcessId(GetForegroundWindow(), &pid);
    if (pid != GetCurrentProcessId() || !g_vehicles)
        return;
    bool session = false;
    for (int i = 0; i < 32 && !session; i++) {
        uint64_t v = 0;
        session = kizi::read(g_vehicles + i * 8, v) && v && human(v) && body_of(v);
    }
    if (!session)
        return;
    int now = 1;
    for (int k = 0; k < 3; k++)
        if (CRASH_LEVELS[k] == level_name())
            now = k;
    int next = (now + 1) % 3;
    preset_summary(preset_index());
    g_level_key = next;
    g_preset_shown = GetTickCount() | 1;
    swofty::log("[ai-tweaks] P key: Crashes %.*s (until the game closes)", int(CRASH_LEVELS[next].size()), CRASH_LEVELS[next].data());
}

void worker()
{
    for (int tick = 0;; tick++) {
        Sleep(50);
        preset_key();
        notice_tick();
        track_jumps();
        aero_compute();
        watch_breaks();
        if (tick % 600 == 0 && (g_aero_lowered || g_aero_wanted)) {
            swofty::log("[ai-tweaks] aero loss, last 30 s: %d slide(s) seen (largest slip %.0f deg), %d with less downforce, lowest %.0f %% "
                        "of the car's own", g_aero_wanted.exchange(0), double(g_aero_beta_max.exchange(0)), g_aero_lowered.exchange(0),
                        double(g_aero_min.exchange(1.0f)) * 100.0);
        }
        if (tick % 600 == 0 && (g_blowovers || g_aoa_max > 5.0f || g_snap_count))
            swofty::log("[ai-tweaks] crashes (%.*s), last 30 s: %d nose-up blow-over(s), steepest nose up %.0f deg at %.0f km/h; %d "
                        "snap(s) after a rear break", int(level_name().size()), level_name().data(), g_blowovers.exchange(0),
                        double(g_aoa_max.exchange(0)), double(g_aoa_kmh.exchange(0)), g_snap_count.exchange(0));
        if (tick % 600 == 0 && g_flight_n)
            swofty::log("[ai-tweaks] flights, last 30 s: %d car(s) flew (highest %.2f m, longest %.2f s), put back on the curve "
                        "%d time(s) (crashes %.*s)", g_flight_n.exchange(0), double(g_fly_high.exchange(0)),
                        double(g_fly_long.exchange(0)), g_fly_held.exchange(0), int(level_name().size()), level_name().data());
        if (tick % 600 == 0 && g_ext_breaks)
            swofty::log("[ai-tweaks] rear breaks by the game / Terminal Damage, last 30 s: %d (snap and slide added)",
                        g_ext_breaks.exchange(0));
        if (tick % 600 == 0 && g_slide_n)
            swofty::log("[ai-tweaks] slides, last 30 s: %d car(s) slid on (farthest %.0f m, most %.1f turn(s)) (crashes %.*s)",
                        g_slide_n.exchange(0), double(g_slide_far.exchange(0)), double(g_slide_turns.exchange(0)),
                        int(level_name().size()), level_name().data());
        if (tick % 40 == 5)
            equal_damage_tick(), front_tough_tick();
        if (tick % 20 == 0)
            apply();
        if (contact_log.get())
            log_contacts();
        if (tick % 20 == 0)
            find_ai_hit();
        if (tick % 600 == 0 && g_spins) {
            swofty::log("[ai-tweaks] contact spin, last 30 s: %d AI car(s) given spin, largest %.0f deg/s",
                        g_spins.exchange(0), g_spin_max.exchange(0) * 57.3f);
        }
        tick_traces();
        if (tick % 600 == 0 && g_loose)
            swofty::log("[ai-tweaks] loose wheels, last 30 s: %d wheel(s) came off without tethers", g_loose.exchange(0));
        if (tick % 600 == 0 && (g_rear_rolls || g_wing_on_tyre))
            swofty::log("[ai-tweaks] tyre hits, last 30 s: %d wheel-on-wheel hit(s) at speed (%d wing-on-tyre, no break), %d "
                        "suspension(s) broken, %d kick(s) (largest sideways %.1f m/s), %d launch(es) (largest %.1f m/s up)",
                        g_rear_rolls.exchange(0), g_wing_on_tyre.exchange(0), g_rear_breaks.exchange(0), g_kicks.exchange(0),
                        double(g_kick_max.exchange(0)), g_launches.exchange(0), double(g_launch_max.exchange(0)));
        if (tick % 6000 == 0)
            for (int k = 0; k < 4; k++)
                preset_summary(k);
        if (tick % 600 == 0 && g_punctures)
            swofty::log("[ai-tweaks] contact punctures, last 30 s: %d tyre(s)", g_punctures.exchange(0));
        if (tick % 600 == 0 && g_lifts)
            swofty::log("[ai-tweaks] wheel lift, last 30 s: %d car(s) lifted, largest %.1f m/s, steepest contact normal %.2f",
                        g_lifts.exchange(0), g_lift_max.exchange(0), g_ny_max.exchange(0));
        if (tick % 600 == 0 && (g_ai_ai || g_ai_you)) {
            swofty::log("[ai-tweaks] contact log, last 30 s: %d AI-AI touch(es), %d you-AI", g_ai_ai, g_ai_you);
            g_ai_ai = g_ai_you = 0;
        }
    }
}

}  // namespace

void sim_lifted(uint64_t body, float dv)           // from kizi_contact_sim.hpp's hook (physics thread, g_spin_mx not held)
{
    std::lock_guard<std::mutex> lock(g_spin_mx);
    lift_noted(body, dv);
}

SWOFTY_ON_CHANGE(classic_strength, std::string_view)   // the preset picks the wheel push (0.9.11)
{
    g_push_key = -1;
    apply();
}

SWOFTY_ON_LOAD
{
    bool label = swofty::overlay::available() && swofty::overlay::on_paint(paint_preset) == swofty::overlay::Result::ok;
    swofty::log("[ai-tweaks] P key preset label %s", label ? "ready" : "not available (overlay off) - the log still shows the preset");
    g_slot[0] = slot_of(ASSIST_B, "ai-tweaks: contact assist (inertia)");
    g_slot[1] = slot_of(ASSIST_A, "ai-tweaks: contact assist (velocity)");
    bool eq = install_equal_weight(kizi::scan_code(ASSIST_A, "ai-tweaks: contact mass ratio"), g_slot[1]);
    swofty::log("[ai-tweaks] contact hook %s", eq ? "installed" : "NOT installed");
    if (uintptr_t hit = kizi::scan_code("48 8D 05 ?? ?? ?? ?? 48 8B 0C F8 48 85 C9 0F 84 ?? ?? ?? ?? 4C 8B B1",
                                        "ai-tweaks: vehicle table")) {
        int32_t d = 0;
        if (kizi::read(hit + 3, d))
            g_vehicles = hit + 7 + d;
    }
    if (uintptr_t hit = kizi::scan_code("4C 8B 81 ?? ?? ?? ?? C4 C1 7A 10 40 48 C4 C1 7A 10 48 34",
                                        "ai-tweaks: front wing field"))
        kizi::read(hit + 3, g_wing_field);
    swofty::log("[ai-tweaks] contact log %s (%s)", g_ring ? "ready" : "NOT available", contact_log.get() ? "on" : "off");
    install_loose_wheels();
    read_td_settings();
    g_add_puncture = reinterpret_cast<AddPunctureFn>(
        kizi::scan_code("41 57 41 56 41 55 41 54 56 57 55 53 48 83 EC 28 4C 89 C7 89 D3 48 89 CE 48 8B 49 28 FF 15",
                        "ai-tweaks: add puncture"));
    g_apply_impulse = reinterpret_cast<ApplyImpulseFn>(
        kizi::scan_code("56 48 83 EC 70 C5 78 29 54 24 60 C5 78 29 4C 24 50 C5 78 29 44 24 40 C5 F8 29 7C 24 30 C5 F8 29 74 "
                        "24 20 48 89 CE 8B 81 6C 04 00 00",
                        "ai-tweaks: apply impulse"));
    swofty::log("[ai-tweaks] wheel lift (game impulse) %s", g_apply_impulse ? "ready" : "NOT available");
    swofty::log("[ai-tweaks] contact punctures %s (%.*s)", g_add_puncture ? "ready" : "NOT available",
                (int)puncture.get().size(), puncture.get().data());
    if (uintptr_t hit = kizi::scan_code(WHEEL, "ai-tweaks: wheel-to-wheel push")) {
        int32_t disp = 0;
        if (kizi::read(hit + 4, disp))
            g_params = hit + 8 + disp;
    }
    swofty::log("[ai-tweaks] contact softening %s, wheel-to-wheel push %s; softening %.*s, push %.*s",
                g_slot[0] && g_slot[1] ? "found" : "NOT found", g_params ? "found" : "NOT found",
                (int)softening.get().size(), softening.get().data(), (int)wheel_push.get().size(),
                wheel_push.get().data());
    apply();
    std::thread(worker).detach();
    if (g_ring)
        std::thread(spin_worker).detach();
    swofty::log("[ai-tweaks] contact spin %s (%.*s, %.*s)", g_ring ? "ready" : "NOT available",
                (int)contact_spin.get().size(), contact_spin.get().data(), (int)spin_who.get().size(),
                spin_who.get().data());
    sim::start();                                  // its hook does nothing until Physics mode is Realistic
    {
        auto r = swofty::events::on_frame(&aero_frame, nullptr);
        swofty::log("[ai-tweaks] aero (sideways + nose up): %s", r == swofty::events::Result::ok ? "ready" : "frame callback refused");
    }
    swofty::log("[ai-tweaks] physics mode: %.*s, crashes %.*s", (int)physics_mode.get().size(), physics_mode.get().data(),
                (int)crashes.get().size(), crashes.get().data());
}

SWOFTY_ADDON()
