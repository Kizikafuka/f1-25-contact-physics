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
// Hotkeys (game window focused): Ctrl+F9 softening on/off, Ctrl+F10 cycles the wheel-to-wheel push (Classic only). Each beeps;
// a message also shows in the game's race notification.
#include <algorithm>
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
#include "osd.hpp"

SWOFTY_SETTING(std::string_view, softening, "off");
SWOFTY_SETTING(std::string_view, wheel_push, "medium");
SWOFTY_SETTING(bool, osd_message, true);
SWOFTY_SETTING(bool, contact_log, false);
SWOFTY_SETTING(std::string_view, contact_spin, "strong");
SWOFTY_SETTING(std::string_view, spin_who, "all");
SWOFTY_SETTING(std::string_view, reaction, "scaled");
SWOFTY_SETTING(std::string_view, twitch, "medium");
SWOFTY_SETTING(std::string_view, spin_target, "both");
SWOFTY_SETTING(std::string_view, rear_boost, "strong");
SWOFTY_SETTING(std::string_view, wheel_lift, "strong");
SWOFTY_SETTING(std::string_view, puncture, "sometimes");
SWOFTY_SETTING(bool, puncture_you, false);
// Physics mode. 0.6.1: the rigid-body response of common/kizi_contact_sim.hpp (impulse from both cars' mass and inertia at the
// contact point, bounce, friction, rolling tyres rubbing, AI cars pushed too) is THE contact physics; the option is no longer
// in the menu, so this default always applies. "classic" (the made-up spin / twitch / rear-hit / lift / wheel push below,
// 0.5.x behaviour) is kept in the code only so it can come back by re-adding the option.
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

// --- AI reaction time ------------------------------------------------------------------------------------------
// Every car has a ControllerSet (+0x28 its AI controller). Each frame the AI controller's update (vtable slot 1)
// copies the AI's decision for car [ctrl+0x18] into the car's input record [ctrl+8]: +0x0c throttle, +0x14 steering,
// +0x18 brake (read live). All AI controllers get a copy of their vtable whose slot 1 calls the game's update and then,
// while that car is "stunned" after a hit, puts the steering back to what it was at the hit - the driver has not
// reacted yet, so the car turns with the hit instead of being caught on the same frame.
uintptr_t g_sets_global;                           // ControllerSet manager pointer
uint8_t g_sets_vec;                                // manager + this = vector of ControllerSet*
using CtrlUpdate = uint64_t (*)(void *);
CtrlUpdate g_ctrl_update;
uint64_t *g_ctrl_vt;                               // our vtable copy (slot 0 at index 1; index 0 = RTTI locator)
uintptr_t g_ctrl_vt_orig;
struct Stun {
    std::atomic<DWORD> until{0};
    std::atomic<float> steer{0};
};
Stun g_stun[32];
std::atomic<int> g_stuns{0};

uint64_t ctrl_update_hook(void *self)
{
    uint64_t r = g_ctrl_update(self);
    auto c = static_cast<uint8_t *>(self);
    uint8_t idx = c[0x18];
    if (idx < 32 && g_stun[idx].until.load() && int(g_stun[idx].until.load() - GetTickCount()) > 0)
        if (auto out = *reinterpret_cast<uint8_t **>(c + 8))
            *reinterpret_cast<float *>(out + 0x14) = g_stun[idx].steer;
    return r;
}

// How long the AI keeps its steering after a hit. Fixed (0.2 / 0.4 / 0.6 s), or "scaled": a light nudge is caught almost at
// once and a big hit takes longer, from 0.1 s at a closing speed of 1 m/s or less to 1.0 s at 10 m/s or more (smooth curve).
DWORD reaction_ms(float impact)
{
    std::string_view v = reaction.get();
    if (v == "scaled") {
        float t = std::clamp((impact - 1.0f) / 9.0f, 0.0f, 1.0f);
        return DWORD(100.0f + 900.0f * t * t * (3.0f - 2.0f * t));
    }
    return v == "0.2" ? 200 : v == "0.4" ? 400 : v == "0.6" ? 600 : 0;
}
std::atomic<uint32_t> g_stun_ms_sum{0}, g_stun_ms_max{0};
std::atomic<float> g_impact_max{0};

// The AI controller of car idx, or 0.
uintptr_t ai_ctrl(int idx)
{
    uint64_t mgr = 0, b = 0, e = 0, set = 0, ai = 0;
    if (!g_sets_global || idx < 0 || !kizi::read(g_sets_global, mgr) || !mgr || !kizi::read(mgr + g_sets_vec, b) ||
        !kizi::read(mgr + g_sets_vec + 8, e) || b + 8 * idx >= e || !kizi::read(b + 8 * idx, set) || !set ||
        !kizi::read(set + 0x28, ai))
        return 0;
    return ai;
}

// Put our vtable on every AI controller that still has the game's (new sessions make new controllers).
void wrap_controllers()
{
    for (int i = 0; i < 32; i++) {
        uintptr_t ai = ai_ctrl(i);
        uint64_t vt = 0;
        if (!ai || !kizi::read(ai, vt) || !vt)
            continue;
        if (!g_ctrl_vt) {                          // first controller: make the copy from its vtable
            uint64_t slot1 = 0;
            if (!kizi::in_image(vt) || !kizi::read(vt + 8, slot1) || !kizi::in_image(slot1))
                return;
            auto copy = new uint64_t[17];
            if (!kizi::read(vt - 8, copy, 17 * 8))
                return;
            g_ctrl_update = reinterpret_cast<CtrlUpdate>(slot1);
            copy[2] = reinterpret_cast<uint64_t>(&ctrl_update_hook);
            g_ctrl_vt_orig = vt, g_ctrl_vt = copy;
        }
        if (vt == g_ctrl_vt_orig) {
            uint64_t nv = reinterpret_cast<uint64_t>(g_ctrl_vt + 1);
            kizi::write(ai, &nv, 8);
        }
    }
}

void stun(int idx, float impact)
{
    DWORD ms = reaction_ms(impact);
    uintptr_t ai = ms ? ai_ctrl(idx) : 0;
    uint64_t out = 0;
    float steer = 0;
    if (!ai || idx >= 32 || !kizi::read(ai + 8, out) || !out || !kizi::read(out + 0x14, steer))
        return;
    if (!g_stun[idx].until.load() || int(g_stun[idx].until.load() - GetTickCount()) <= 0)
        g_stun[idx].steer = steer;                 // a new stun freezes the steering of this moment
    g_stun[idx].until = GetTickCount() + ms;
    g_stuns++;
    g_stun_ms_sum += ms;
    if (ms > g_stun_ms_max)
        g_stun_ms_max = ms;
    if (impact > g_impact_max)
        g_impact_max = impact;
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
    for (;;) {
        Sleep(1);
        DWORD t = GetTickCount();
        std::lock_guard<std::mutex> lock(g_spin_mx);
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

std::string upper(std::string s)
{
    for (auto &c : s)
        c = char(toupper(uint8_t(c)));
    return s;
}

void worker()
{
    bool down9 = false, down10 = false;
    for (int tick = 0;; tick++) {
        Sleep(50);
        if (tick % 20 == 0)
            apply();
        if (contact_log.get())
            log_contacts();
        if (tick % 20 == 0)
            wrap_controllers();
        if (tick % 600 == 0 && g_stuns)
        {
            int n = g_stuns.exchange(0);
            swofty::log("[ai-tweaks] reaction time, last 30 s: %d AI car(s) slowed after a hit, average %u ms, longest %u ms, "
                        "hardest hit %.1f m/s", n, n ? g_stun_ms_sum.exchange(0) / unsigned(n) : 0u, g_stun_ms_max.exchange(0),
                        double(g_impact_max.exchange(0)));
        }
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
        bool ctrl = GetAsyncKeyState(VK_CONTROL) & 0x8000;
        bool now9 = ctrl && (GetAsyncKeyState(VK_F9) & 0x8000), now10 = ctrl && (GetAsyncKeyState(VK_F10) & 0x8000);
        if (now9 && !down9) {
            bool off = !soft_off();
            g_soft_key = off ? 1 : 0;
            apply();
            osd::show(off ? "CONTACT SOFTENING: OFF" : "CONTACT SOFTENING: GAME", off ? "REALISTIC CONTACT" : "");
            Beep(off ? 1000 : 500, 150);
        }
        if (now10 && !down10 && !sim::cfg::active()) {   // the wheel push is a Classic feature
            int t = (push_index() + 1) % LEVELS;
            g_push_key = t;
            apply();
            osd::show("WHEEL PUSH: " + upper(PUSH[t].name),
                      t ? "LEVEL " + std::to_string(t) + " OF " + std::to_string(LEVELS - 1) : "");
            if (t == 0)
                Beep(400, 300);
            for (int i = 0; i < t; i++)
                Beep(1000 + 150 * t, 100), Sleep(70);
        }
        down9 = now9, down10 = now10;
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

SWOFTY_ON_CHANGE(osd_message, bool on)
{
    osd::enabled = on;
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
    // ControllerSet table (vehicle input code): mov rcx,[rip+G] ... mov rcx,[rcx+V]; mov rcx,[rcx+rdx*8]; mov edx,[rcx+M]
    if (uintptr_t hit = kizi::scan_code("48 8B 49 ?? 48 8B 0C D1 8B 51 ?? 33 51 ??", "ai-tweaks: controller sets")) {
        for (uintptr_t a = hit - 8; a >= hit - 32; a--) {
            uint8_t op[3] = {};
            int32_t d = 0;
            if (kizi::read(a, op, 3) && op[0] == 0x48 && op[1] == 0x8B && op[2] == 0x0D && kizi::read(a + 3, d)) {
                g_sets_global = a + 7 + d;
                kizi::read(hit + 3, g_sets_vec);
                break;
            }
        }
    }
    swofty::log("[ai-tweaks] AI reaction time %s (%.*s s)", g_sets_global ? "ready" : "NOT available",
                (int)reaction.get().size(), reaction.get().data());
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
    osd::enabled = osd_message.get();
    {
        bool box = osd::start();
        swofty::log("[ai-tweaks] on-screen messages: %s", swofty::hud::available() ? "game race notification" : box ? "box on screen" : "not available on this launcher (beeps only)");
    }
    apply();
    std::thread(worker).detach();
    if (g_ring)
        std::thread(spin_worker).detach();
    swofty::log("[ai-tweaks] contact spin %s (%.*s, %.*s)", g_ring ? "ready" : "NOT available",
                (int)contact_spin.get().size(), contact_spin.get().data(), (int)spin_who.get().size(),
                spin_who.get().data());
    sim::start();                                  // its hook does nothing until Physics mode is Realistic
    swofty::log("[ai-tweaks] physics mode: %.*s", (int)physics_mode.get().size(), physics_mode.get().data());
}

SWOFTY_ADDON()
