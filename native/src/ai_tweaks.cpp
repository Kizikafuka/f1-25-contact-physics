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
#include <map>
#include <mutex>
#include <vector>
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>
#include <thread>
#include "kizi_mem.hpp"
#include <swofty/events.hpp>
#include <swofty/systems/cars.hpp>
#include "osd.hpp"

SWOFTY_SETTING(std::string_view, softening, "off");
SWOFTY_SETTING(std::string_view, wheel_push, "medium");
SWOFTY_SETTING(bool, contact_log, false);
SWOFTY_SETTING(std::string_view, contact_spin, "strong");
SWOFTY_SETTING(std::string_view, spin_who, "all");
SWOFTY_SETTING(std::string_view, twitch, "medium");
SWOFTY_SETTING(std::string_view, spin_target, "both");
SWOFTY_SETTING(std::string_view, rear_boost, "strong");
SWOFTY_SETTING(std::string_view, wheel_lift, "strong");
SWOFTY_SETTING(std::string_view, puncture, "sometimes");
SWOFTY_SETTING(bool, puncture_you, false);
SWOFTY_SETTING(std::string_view, jump_break, "sometimes");
SWOFTY_SETTING(std::string_view, jump_damage, "damage");
SWOFTY_SETTING(bool, jump_you, false);
SWOFTY_SETTING(bool, equal_damage, true);
SWOFTY_SETTING(bool, td_defer, true);
SWOFTY_SETTING(bool, walls, true);
SWOFTY_SETTING(std::string_view, damage_boost, "normal");
SWOFTY_SETTING(std::string_view, aero_loss, "realistic");
// Physics mode ("Physics: contact model"). "realistic" = Rigid body: the response of common/kizi_contact_sim.hpp (impulse from
// both cars' mass and inertia at the contact point, bounce, friction, rolling tyres rubbing, AI cars pushed too); the only
// model in 0.6.1 - 0.7.3. "classic" = the 0.5.x contact (the spin / twitch / rear-hit / lift / wheel push below), back as a
// choice in 0.8.0 because players missed the flying on tyre contact. Walls, aero loss and damage work with both.
SWOFTY_SETTING(std::string_view, physics_mode, "realistic");
SWOFTY_SETTING(std::string_view, exaggerate, "off");
SWOFTY_SETTING(std::string_view, restitution, "medium");
SWOFTY_SETTING(std::string_view, friction, "medium");
SWOFTY_SETTING(std::string_view, engine_response, "add");
SWOFTY_SETTING(bool, tyre_spin, true);
SWOFTY_SETTING(bool, ai_impulse, true);

namespace sim::cfg {
inline bool active() { return physics_mode.get() == "realistic"; }
inline std::string_view normal_mode() { return "full"; }
inline std::string_view apply_to() { return spin_who.get(); }
inline bool contact_log() { return ::contact_log.get(); }
inline std::string_view restitution() { return ::restitution.get(); }
inline std::string_view friction() { return ::friction.get(); }
inline std::string_view separation() { return "low"; }
inline std::string_view engine_response() { return ::engine_response.get(); }
inline std::string_view exaggerate() { return ::exaggerate.get(); }
inline bool tyre_spin() { return ::tyre_spin.get(); }
inline bool ai_impulse() { return ::ai_impulse.get(); }
inline bool walls() { return ::walls.get(); }
inline float damage_boost()
{
    std::string_view v = ::damage_boost.get();
    return v == "normal" ? 1.0f : v == "high" ? 2.0f : 0.0f;
}
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
        swofty::log("[ai-tweaks] contact %s: closing %.1f km/h, %d step(s) over %lu ms, game ratio %.2f..%.2f | %s | %s",
                    you ? "YOU-AI" : "AI-AI", t.closing, t.steps, t.last - t.first, t.ratio_min, t.ratio_max,
                    car(t.c[0]).c_str(), car(t.c[1]).c_str());
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
    return v == "rare" ? 0.15f : v == "sometimes" ? 0.35f : v == "often" ? 0.7f : 0.0f;
}

uint32_t g_rng = 2463534242u;
float rnd01()
{
    g_rng ^= g_rng << 13, g_rng ^= g_rng >> 17, g_rng ^= g_rng << 5;
    return float(g_rng & 0xFFFFFF) / float(0x1000000);
}

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
    if (!ha)
        stun(car_id(a), impact);
    if (!hb)
        stun(car_id(b), impact);
    if (float pc = puncture_chance(); pc > 0) {
        float pt[3], va2[3], vb2[3];
        uint64_t b1 = body_of(a), b2 = body_of(b);
        if (b1 && b2 && kizi::read(contact + 0x70, pt, 12) && kizi::read(b1 + 0x220, va2, 12) &&
            kizi::read(b2 + 0x220, vb2, 12)) {
            float kmh = len({va2[0] - vb2[0], va2[1] - vb2[1], va2[2] - vb2[2]}) * 3.6f;
            float c = pc * std::clamp((kmh - 10.0f) / 50.0f, 0.0f, 1.0f);   // under 10 km/h never, full chance from 60 km/h
            if (c > 0) {
                if (!ha || puncture_you)
                    puncture_wheel(a, pt, c);
                if (!hb || puncture_you)
                    puncture_wheel(b, pt, c);
            }
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
    for (;;) {
        Sleep(idle ? 25 : 1);                      // 1 ms only while a Classic spin is being held (never in the default physics)
        DWORD t = GetTickCount();
        std::lock_guard<std::mutex> lock(g_spin_mx);
        idle = g_holds.empty();
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

float aero_floor()
{
    std::string_view v = aero_loss.get();
    return v == "realistic" ? 0.3f : v == "strong" ? 0.12f : 1.0f;
}

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
                    float sp = len(vh), fl = len(fh);
                    if (sp > 12.0f && fl > 0.3f) {
                        float c = std::clamp(dot(vh, fh) / (sp * fl), -1.0f, 1.0f);
                        float beta = std::acos(c) * 57.2958f;          // slip angle, 0 .. 180 degrees
                        float t = std::clamp((beta - 15.0f) / 75.0f, 0.0f, 1.0f);
                        want = floor < 1.0f ? 1.0f - (1.0f - floor) * t * t * (3.0f - 2.0f * t) : 1.0f;
                        if (beta > g_aero_beta_max)
                            g_aero_beta_max = beta;
                        if (want < 1.0f && g_aero_want[i].load() == 1.0f)
                            g_aero_wanted++;       // a slide starts (counted even if the field can't be set)
                    }
                }
            } else
                g_aero_body[i] = 0;
        }
        g_aero_want[i] = want;
    }
}

void aero_frame(double, void *)
{
    static double acc = 0;
    static bool checked = false;
    acc += 1.0 / 60.0;                             // the frame tick passes dt; a fixed step is enough for a 20 Hz rate
    if (acc < 0.05 || g_aero_disabled || !g_vehicles)
        return;
    acc = 0;
    if (!live::available())
        return;
    if (!checked) {                                // the live car index must be our vehicle table index (checked once cars exist)
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
// Terminal Damage (nick.terminal-damage-probe) has its own suspension model ("Suspension damage from hits") that replaces the
// game's suspension damage. With "leave suspension to Terminal Damage" on and its DLL loaded, this addon writes no suspension
// damage of its own (no jump damage, no suspension factors for AI cars).
bool td_owns_suspension()
{
    return td_defer.get() && GetModuleHandleA("terminal_damage_probe.dll") != nullptr;
}

// The game applies its damage setting with a different factor for your car and for AI cars (research/DAMAGE_RE.md,
// 0x142f182e0, vehicle flag bit 3): wing component +0x40/+0x44, suspension component +0x128/+0x12c, each body part +0x50/+0x54.
// "Equal damage" copies your car's factors onto every AI car (re-checked every 2 s, the game sets them again per session),
// so an AI car takes as much damage from a hit as your car would.
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
    bool have_susp = my_susp && kizi::read(my_susp + 0x128, susp, 8) && !td_owns_suspension();
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
    if (td_owns_suspension()) {                    // Terminal Damage runs its own suspension model
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
    if (to > now)
        kizi::write(at, &to, 4);
    swofty::log("[ai-tweaks] jump: car %d (%s) flew %.2f s, landed at %.1f m/s -> %s suspension %s (%.2f -> %.2f of %.2f)", idx,
                you ? "YOU" : "AI", secs, landing, names[corner], brk ? "BROKEN" : "damaged", now, std::max(now, to), thr);
}

void track_jumps()
{
    if (!g_vehicles || jump_chance() <= 0)
        return;
    DWORD t = GetTickCount();
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
            f.fall = a < -7.0f ? f.fall + 1 : 0;
            f.vy_max = f.fall ? std::max(f.vy_max, f.vy) : f.vy;       // vertical speed when the fall began (did it rise?)
            if (f.fall >= 3)
                f.air = true, f.start = t - 150, f.vy_min = vy, f.ground = 0;
        } else {
            f.vy_min = std::min(f.vy_min, vy);
            f.ground = a > -3.0f ? f.ground + 1 : 0;
            if (f.ground >= 2) {
                landed(i, f, float(t - f.start - 100) / 1000.0f);
                f.air = false, f.fall = 0;
            } else if (t - f.start > 6000)
                f.air = false, f.fall = 0;     // something else (a long fall off the world): give up
        }
        f.vy = vy, f.last = t;
    }
}

void worker()
{
    for (int tick = 0;; tick++) {
        Sleep(50);
        track_jumps();
        aero_compute();
        if (tick % 600 == 0 && (g_aero_lowered || g_aero_wanted)) {
            swofty::log("[ai-tweaks] aero loss, last 30 s: %d slide(s) seen (largest slip %.0f deg), %d with less downforce, lowest %.0f %% "
                        "of the car's own", g_aero_wanted.exchange(0), double(g_aero_beta_max.exchange(0)), g_aero_lowered.exchange(0),
                        double(g_aero_min.exchange(1.0f)) * 100.0);
        }
        if (tick % 40 == 5)
            equal_damage_tick();
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

SWOFTY_ON_CHANGE(softening, std::string_view)
{
    g_soft_key = -1;
    apply();
}

SWOFTY_ON_CHANGE(wheel_push, std::string_view)
{
    g_push_key = -1;
    apply();
}

SWOFTY_ON_LOAD
{
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
        swofty::log("[ai-tweaks] aero loss when sideways: %s (%.*s)", r == swofty::events::Result::ok ? "ready" : "frame callback refused",
                    (int)aero_loss.get().size(), aero_loss.get().data());
    }
    swofty::log("[ai-tweaks] physics mode: %.*s", (int)physics_mode.get().size(), physics_mode.get().data());
}

SWOFTY_ADDON()
