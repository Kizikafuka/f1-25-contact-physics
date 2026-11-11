// SPDX-License-Identifier: MIT
// Contact Sim (TEST ONLY), shared by two variants (KIZI_SIM_HEAVY 0 = light, 1 = heavy). See research/CONTACT_HANDLER_RE.md.
//
// The game's car-to-car contact handler (0x142f04040 on the Steam build we read) is a contact MODIFIER: it edits one contact
// record (rsi) before the engine's solver reads it. We hook the handler's common epilogue, which every path reaches after all
// of the game's own edits, with a stub that calls on_contact(contact) on the physics thread.
//   contact+0x32 flag byte (bit 0 = the game's own "ignore this contact"), +0x38/+0x40 the two collision objects,
//   +0x48/+0x50 type tags ('CARW' wheel, 'CARB' body), +0x64 mass ratio, +0x70 contact point, +0x80 normal.
//   car body (see ai_tweaks.cpp): +0x1c0 position, +0x210 angular velocity, +0x220 velocity, +0x378 -> centre of mass,
//   +0xd0/+0xe0/+0xf0 world inverse inertia rows, +0x438 inverse mass, +0x46c flags (1 linear, 2 angular).
//   wheels: [[vehicle+0x580]+0x70+8i], +0x2f0 world position.
// LIGHT: only rewrites the contact normal of a tyre-on-tyre contact from the two wheel centres (keeps the vertical part the
//   game's assists remove), and leaves the engine's response alone.
// HEAVY: also computes its own rigid-body response (restitution, Coulomb friction, both inertias) for car-car contacts, applies
//   it with the game's own apply-impulse function (0x1440c12e0) and, optionally, tells the engine to ignore the contact.
#pragma once
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>
#include <map>
#include <thread>

#include "kizi_mem.hpp"

#ifndef KIZI_SIM_HEAVY
#define KIZI_SIM_HEAVY 0
#endif
#ifndef SIM_TAG
#if KIZI_SIM_HEAVY
#define SIM_TAG "[contact-sim-heavy]"
#else
#define SIM_TAG "[contact-sim-light]"
#endif
#endif

// Settings are read through sim::cfg. A test addon declares its own options here; with KIZI_SIM_EMBED (Contact Physics,
// "Physics mode: Realistic") the host addon defines sim::cfg itself and calls sim::start() from its own load.
#ifndef KIZI_SIM_EMBED
SWOFTY_SETTING(std::string_view, normal_mode, "full");
SWOFTY_SETTING(std::string_view, apply_to, "all");
SWOFTY_SETTING(bool, contact_log, true);
#if KIZI_SIM_HEAVY
SWOFTY_SETTING(std::string_view, restitution, "medium");
SWOFTY_SETTING(std::string_view, friction, "medium");
SWOFTY_SETTING(std::string_view, separation, "low");
SWOFTY_SETTING(std::string_view, engine_response, "add");
SWOFTY_SETTING(std::string_view, exaggerate, "off");
SWOFTY_SETTING(bool, tyre_spin, true);
SWOFTY_SETTING(bool, ai_impulse, true);
#endif
namespace sim::cfg {
inline bool active() { return true; }
inline std::string_view normal_mode() { return ::normal_mode.get(); }
inline std::string_view apply_to() { return ::apply_to.get(); }
inline bool contact_log() { return ::contact_log.get(); }
#if KIZI_SIM_HEAVY
inline std::string_view restitution() { return ::restitution.get(); }
inline std::string_view friction() { return ::friction.get(); }
inline std::string_view separation() { return ::separation.get(); }
inline std::string_view engine_response() { return ::engine_response.get(); }
inline std::string_view exaggerate() { return ::exaggerate.get(); }
inline bool tyre_spin() { return ::tyre_spin.get(); }
inline bool ai_impulse() { return ::ai_impulse.get(); }
#endif
}  // namespace sim::cfg
#endif

namespace sim {

struct V3 {
    float x = 0, y = 0, z = 0;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float len(V3 a) { return std::sqrt(dot(a, a)); }
inline bool finite(V3 a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }
inline V3 norm(V3 a)
{
    float l = len(a);
    return l > 1e-6f ? a * (1.0f / l) : V3{};
}
// Direct reads and writes for the physics thread: ReadProcessMemory is a system call, and with ~100 reads per contact record
// it cost frames. These only check for a null / absurd address; every pointer used is one the game itself dereferences here.
inline bool mread(uint64_t a, void *out, size_t n)
{
    if (a < 0x10000 || a > 0x00007FFFFFFF0000ull)
        return false;
    memcpy(out, reinterpret_cast<const void *>(a), n);
    return true;
}
template <class T> inline bool mread(uint64_t a, T &out) { return mread(a, &out, sizeof out); }
inline bool mwrite(uint64_t a, const void *data, size_t n)
{
    if (a < 0x10000 || a > 0x00007FFFFFFF0000ull)
        return false;
    memcpy(reinterpret_cast<void *>(a), data, n);
    return true;
}

inline bool rd3(uint64_t a, V3 &v)
{
    float f[3];
    if (!mread(a, f, 12))
        return false;
    v = {f[0], f[1], f[2]};
    return finite(v);
}

uint64_t g_last_slot;                              // the handler's "last contact" global (rip-relative in its first compare)
using ApplyImpulseFn = void (*)(void *body, const float *impulse, const float *point);
ApplyImpulseFn g_apply;

struct Car {
    uint64_t veh = 0, body = 0, wheel[4] = {};
    float invm = 0;
    V3 pos, com, lin, ang, row[3], up;
    bool human = false;
    uint8_t vflag = 0;
};

uint64_t vehicle_of(uint64_t obj)
{
    uint64_t p = obj;
    for (uint32_t off : {0x150u, 0x18u, 0x108u, 0x458u})
        if (!p || !mread(p + off, p))
            return 0;
    uint64_t veh = 0;
    return p && mread(p + 8, veh) ? veh : 0;
}

std::atomic<float> g_dbg_invm{0};                  // last values seen while loading a car (for the log)
std::atomic<uint32_t> g_dbg_flags{0}, g_dbg_vflag{0};

// 0 = loaded, else the step that failed (counted in the 30 s summary): 1 physics chain, 2 inverse mass, 3 body flags,
// 4 velocity / position / centre of mass, 5 inertia rows, 6 wheels.
int load_car(uint64_t veh, Car &c)
{
    uint64_t phys = 0, rb = 0, comp = 0, set = 0;
    uint32_t flags = 0;
    float mass_factor = 0;
    c.veh = veh;
    if (!mread(veh + 0x558, phys) || !phys || !mread(phys + 0x340, rb) || !rb ||
        !mread(rb + 0x18, c.body) || !c.body)
        return 1;
    if (!mread(c.body + 0x438, c.invm) || !(c.invm > 1e-9f) || !(c.invm < 100.0f))
        return g_dbg_invm = c.invm, 2;
    g_dbg_invm = c.invm;
    if (!mread(c.body + 0x46c, flags) || (flags & 3) == 0 || !mread(veh + 0x130, c.vflag))
        return g_dbg_flags = flags, 3;
    g_dbg_flags = flags, g_dbg_vflag = c.vflag;
    if (!rd3(c.body + 0x220, c.lin) || !rd3(c.body + 0x210, c.ang) || !rd3(c.body + 0x1c0, c.pos) || !rd3(c.body + 0x1a0, c.up) ||
        !mread(c.body + 0x378, comp) || !comp || !rd3(comp, c.com))
        return 4;
    for (int i = 0; i < 3; i++)
        if (!rd3(c.body + 0xd0 + 0x10 * i, c.row[i]))
            return 5;
    if (mread(phys + 0x3e4, mass_factor))
        c.human = mass_factor > 47.5f;
    if (!mread(veh + 0x580, set) || !set)
        return 6;
    for (int i = 0; i < 4; i++)
        if (!mread(set + 0x70 + 8 * i, c.wheel[i]) || !c.wheel[i])
            return 6;
    return 0;
}

V3 inv_inertia(const Car &c, V3 v) { return c.row[0] * v.x + c.row[1] * v.y + c.row[2] * v.z; }
V3 point_velocity(const Car &c, V3 r) { return c.lin + cross(c.ang, r); }
// 1/m_eff of the body along direction d at lever r: invm + d . ((I^-1 (r x d)) x r)
float k_along(const Car &c, V3 r, V3 d) { return c.invm + dot(d, cross(inv_inertia(c, cross(r, d)), r)); }

bool nearest_wheel(const Car &c, V3 p, V3 &centre, float &dist)
{
    dist = 1e9f;
    bool ok = false;
    for (uint64_t w : c.wheel) {
        V3 pos;
        if (!rd3(w + 0x2f0, pos))
            continue;
        float d = len(pos - p);
        if (d < dist)
            dist = d, centre = pos, ok = true;
    }
    return ok;
}

// Surface speed of a ROLLING tyre at point p, relative to its hub (wheel centre c): the tyre spins at w = (up x v_hub) / R,
// so its bottom point is still on the road, its top moves at 2x the car's speed and its back moves UP, its front DOWN.
// In a tyre-on-tyre touch from behind, the front of the chasing car's front tyre moves down while the back of the other car's
// rear tyre moves up: rubber on rubber, the friction between those surfaces lifts the chasing car (the real wheel-over-wheel
// launch). Without the spin both surfaces look still and nothing lifts. R = 0.36 m (an F1 tyre); no wheel slip assumed.
V3 tyre_surface_velocity(const Car &car, V3 c, V3 p)
{
    V3 up = norm(car.up);
    V3 hub = point_velocity(car, c - car.com);
    V3 v = hub - up * dot(hub, up);                // hub speed along the road
    V3 w = cross(up, v) * (1.0f / 0.36f);
    return cross(w, p - c);
}

// --- log ring (filled on the physics thread, printed by the worker) ------------------------------------------------------
struct Sample {
    uint32_t tag_a, tag_b;
    uint8_t flag32;
    bool tyre, human_a, human_b;
    float ratio, ny_old, ny_new, vn, delta_v, vt, mu_used, lift;
    uint64_t body_a, body_b;                       // for the per-touch log (the worker reads their height afterwards)
    float dvy_a, dvy_b;                            // vertical speed we gave each car in this record (m/s)
};
constexpr uint32_t RING = 4096;                    // ~800 records/s, the worker reads every 50 ms
Sample g_ring[RING];
std::atomic<uint32_t> g_ring_next{0};
std::atomic<uint32_t> g_seen{0}, g_carcar{0}, g_tyre{0}, g_rewritten{0}, g_applied{0}, g_ignored{0};
std::atomic<float> g_max_dv{0}, g_max_ny{0}, g_max_lift{0};
std::atomic<uint32_t> g_spin_touches{0}, g_step_capped{0};
// why a record was not handled: 0 no object, 1 no vehicle (wall / ground), 2 same vehicle, 3 car A load, 4 car B load, 5 apply-to,
// 6 contact point / normal, 7 neither car flagged (veh+0x130 bit 3)
std::atomic<uint32_t> g_rej[8];

float normal_strength()
{
    std::string_view v = cfg::normal_mode();
    return v == "half" ? 0.5f : v == "full" ? 1.0f : 0.0f;
}

#if KIZI_SIM_HEAVY
float restitution_value()
{
    std::string_view v = cfg::restitution();
    return v == "low" ? 0.1f : v == "medium" ? 0.25f : v == "high" ? 0.4f : 0.0f;   // cars crumple: low bounce (0.0.8 values, tuned in tests)
}
float friction_scale()
{
    std::string_view v = cfg::friction();
    return v == "low" ? 0.5f : v == "medium" ? 1.0f : v == "high" ? 1.5f : 0.0f;
}
// Testing: makes the own response bigger than real so it can be felt. Scales the bounce (capped at 0.8 of the closing speed)
// and the friction between the touching parts (capped at 2.0), so hits shove and turn cars harder.
float exaggerate_factor()
{
    std::string_view v = cfg::exaggerate();
    return v == "1.5" ? 1.5f : v == "2" ? 2.0f : v == "3" ? 3.0f : 1.0f;
}
float separation_speed()
{
    std::string_view v = cfg::separation();
    return v == "low" ? 0.15f : v == "medium" ? 0.3f : v == "high" ? 0.6f : 0.0f;
}
#endif

// Called on the game's physics thread from the stub, right before the handler returns.
void on_contact(uint64_t contact)
{
    if (!cfg::active())
        return;                                    // embedded and the host's physics mode is not Realistic
    uint64_t last = 0;
    if (!contact || !mread(g_last_slot, last) || last != contact)
        return;                                    // not a live record (the handler's early exit for a repeated record)
    g_seen++;
    uint64_t oa = 0, ob = 0, ta = 0, tb = 0;
    if (!mread(contact + 0x38, oa) || !mread(contact + 0x40, ob) || !oa || !ob) {
        g_rej[0]++;
        return;
    }
    uint64_t va = vehicle_of(oa), vb = vehicle_of(ob);
    if (!va || !vb) {
        g_rej[1]++;
        return;
    }
    if (va == vb) {
        g_rej[2]++;
        return;
    }
    Car A, B;
    if (load_car(va, A)) {
        g_rej[3]++;
        return;
    }
    if (load_car(vb, B)) {
        g_rej[4]++;
        return;
    }
    if (cfg::apply_to() == "you" && !A.human && !B.human) {
        g_rej[5]++;
        return;
    }
    V3 P, N;
    if (!rd3(contact + 0x70, P) || !rd3(contact + 0x80, N) || len(N) < 0.5f || len(P - A.pos) > 8.0f || len(P - B.pos) > 8.0f) {
        g_rej[6]++;
        return;
    }
    // Vehicle flag bit 3 (veh+0x130) is set on your car only (0xfb vs 0x73 on AI cars): the game's own wheel push only pushes
    // flagged cars. Up to 0.0.6 we copied that, so AI cars never took our impulse and AI-AI touches were skipped. Now every car
    // counts; heavy can still go back to the old rule with "push AI cars" off.
#if KIZI_SIM_HEAVY
    bool flag_rule = !cfg::ai_impulse();
#else
    bool flag_rule = false;
#endif
    if (flag_rule && !(A.vflag & 8) && !(B.vflag & 8)) {
        g_rej[7]++;
        return;
    }
    g_carcar++;
    Sample s{};
    mread(contact + 0x48, ta), mread(contact + 0x50, tb);
    if (ta)
        mread(ta, s.tag_a);
    if (tb)
        mread(tb, s.tag_b);
    mread(contact + 0x32, s.flag32);
    mread(contact + 0x64, s.ratio);
    s.human_a = A.human, s.human_b = B.human;
    s.body_a = A.body, s.body_b = B.body;

    V3 cA, cB;
    float dA = 0, dB = 0;
    bool tyre = nearest_wheel(A, P, cA, dA) && nearest_wheel(B, P, cB, dB) && dA < 0.5f && dB < 0.5f && len(cB - cA) > 0.05f;
    s.tyre = tyre;
    if (tyre)
        g_tyre++;
    V3 old_n = norm(N);
    s.ny_old = std::fabs(old_n.y);

    // Normal: tyre on tyre = the line between the two wheel centres (it points up when one wheel rides on the other).
    // Orientation follows the game's own normal so the engine's convention is kept.
    V3 n_game = old_n;
    float strength = normal_strength();
    V3 n_wheels = tyre ? norm(cB - cA) : V3{};
    if (tyre && dot(n_wheels, old_n) < 0)
        n_wheels = n_wheels * -1.0f;
    s.ny_new = s.ny_old;
    if (tyre && strength > 0) {
        V3 m = norm(old_n * (1.0f - strength) + n_wheels * strength);
        if (finite(m) && len(m) > 0.5f) {
            float f[3] = {m.x, m.y, m.z};
            if (mwrite(contact + 0x80, f, 12)) {
                g_rewritten++;
                n_game = m;
                s.ny_new = std::fabs(m.y);
                if (s.ny_new > g_max_ny)
                    g_max_ny = s.ny_new;
            }
        }
    }

#if KIZI_SIM_HEAVY
    {
        // Own rigid-body response along n (A -> B). The normal of a tyre contact is the wheel line; otherwise the game's.
        V3 n = tyre ? n_wheels : n_game;
        if (dot(n, B.pos - A.pos) < 0)
            n = n * -1.0f;
        V3 rA = P - A.com, rB = P - B.com;
        V3 vrel = point_velocity(A, rA) - point_velocity(B, rB);
        if (tyre && cfg::tyre_spin()) {             // the two tyre surfaces also move (rolling), see tyre_surface_velocity
            V3 sv = tyre_surface_velocity(A, cA, P) - tyre_surface_velocity(B, cB, P);
            sv = sv - n * dot(sv, n);              // only the part along the surface matters (friction)
            if (finite(sv)) {
                vrel = vrel + sv;
                g_spin_touches++;
            }
        }
        float vn = dot(vrel, n);                   // > 0: approaching
        float x = exaggerate_factor();
        float e = std::min(restitution_value() * x, 0.8f), vmin = separation_speed();
        bool keep_engine = cfg::engine_response() != "ignore";
        if (keep_engine)
            vmin = 0;                              // the engine still fixes overlap, so no extra push-apart speed
        float vn_target = -std::max(vmin, vn > 0 ? e * vn : 0.0f);   // wanted normal speed afterwards (separating)
        float k = k_along(A, rA, n) + k_along(B, rB, n);
        s.vn = vn;
        bool step_full = false;
        {
            struct StepSlot {
                uint64_t a, b;
                float x, z;
                int n;
            };
            static StepSlot slots[64];
            StepSlot &st = slots[((A.body >> 4) ^ (B.body >> 6)) & 63];
            if (st.a == A.body && st.b == B.body && st.x == A.pos.x && st.z == A.pos.z)
                step_full = ++st.n > 4;
            else
                st = {A.body, B.body, A.pos.x, A.pos.z, 1};
            if (step_full)
                g_step_capped++;
        }
        if (!step_full && vn > vn_target + (keep_engine ? 0.02f : 0.0f) && k > 1e-6f && std::isfinite(vn)) {
            float dv = std::min(vn - vn_target, 40.0f);       // never change the closing speed by more than 40 m/s per record
            float j = dv / k;
            V3 vt = vrel - n * vn;
            // hot racing slicks on rubber 1.8, carbon bodies 0.6 (0.0.8: the textbook 1.0 / 0.3 felt like nothing in tests, part of
            // each push is lost because the engine also handles the contact)
            float vtl = len(vt), jt = 0, mu = std::min((tyre ? 1.8f : 0.6f) * friction_scale() * x, 3.0f);
            V3 t{};
            if (vtl > 1e-3f && mu > 0) {
                t = vt * (1.0f / vtl);
                float kt = k_along(A, rA, t) + k_along(B, rB, t);
                if (kt > 1e-6f)
                    jt = std::min(vtl / kt, mu * j);
            }
            V3 J = n * j + t * jt;                 // impulse that acts on B; A gets the opposite
            if (finite(J)) {
                alignas(16) float ja[4] = {-J.x, -J.y, -J.z, 0}, jb[4] = {J.x, J.y, J.z, 0};
                alignas(16) float pt[4] = {P.x, P.y, P.z, 1};
                bool push_a = !flag_rule || (A.vflag & 8), push_b = !flag_rule || (B.vflag & 8);
                s.dvy_a = push_a ? -J.y * A.invm : 0.0f;
                s.dvy_b = push_b ? J.y * B.invm : 0.0f;
                if (push_a)
                    g_apply(reinterpret_cast<void *>(A.body), ja, pt);
                if (push_b)
                    g_apply(reinterpret_cast<void *>(B.body), jb, pt);
                g_applied++;
                s.delta_v = dv, s.vt = vtl, s.mu_used = mu;
                float lift = std::fabs(J.y) * std::max(A.invm, B.invm);   // vertical speed change of the lighter side (m/s)
                s.lift = lift;
                if (lift > g_max_lift)
                    g_max_lift = lift;
                if (dv > g_max_dv)
                    g_max_dv = dv;
                if (!keep_engine) {
                    uint8_t f = 0;
                    if (mread(contact + 0x32, f)) {
                        f |= 1;
                        mwrite(contact + 0x32, &f, 1);
                        g_ignored++;
                    }
                }
            }
        } else if (std::isfinite(vn) && k > 1e-6f && cfg::engine_response() == "ignore") {
            uint8_t f = 0;                         // already separating: nothing to push, the engine must not pull either
            if (mread(contact + 0x32, f)) {
                f |= 1;
                mwrite(contact + 0x32, &f, 1);
                g_ignored++;
            }
        }
    }
#endif
    g_ring[g_ring_next++ & (RING - 1)] = s;
}

std::string tag_text(uint32_t t)
{
    char b[5] = {};
    for (int i = 0; i < 4; i++) {
        char c = char((t >> (8 * i)) & 0xFF);
        b[i] = (c >= 32 && c < 127) ? c : '.';
    }
    return b;
}

// Per-touch log: the records of one car pair are grouped into a touch (over when 0.3 s pass without one). For 1.5 s from its
// start the worker reads both cars' height and vertical speed (body +0x1c4 / +0x224) every 50 ms, so the line shows what
// really happened: how far each car rose, its peak vertical speed, and how much vertical speed we gave it.
struct Watch {
    uint64_t body = 0;
    bool you = false;
    float y0 = 0, rise = 0, vy_peak = 0, given = 0;
    bool ok = false;
};
struct Touch {
    Watch c[2];
    DWORD first = 0, last = 0;
    int steps = 0, tyre_steps = 0;
    float ny_max = 0, vn_max = 0;
};

void worker()
{
    uint32_t read_at = g_ring_next;
    int body_left = 10, touch_left = 25;           // lines per 30 s
    std::map<std::pair<uint64_t, uint64_t>, Touch> touches;
    for (int tick = 0;; tick++) {
        Sleep(50);
        DWORD now = GetTickCount();
        if (cfg::contact_log()) {
            uint32_t end = g_ring_next;
            if (end - read_at > RING)
                read_at = end - RING;
            for (; read_at != end; read_at++) {
                const Sample s = g_ring[read_at & (RING - 1)];
                if (!s.body_a || !s.body_b)
                    continue;
                bool swap = s.body_a > s.body_b;
                auto key = swap ? std::make_pair(s.body_b, s.body_a) : std::make_pair(s.body_a, s.body_b);
                auto it = touches.find(key);
                if (it == touches.end()) {
                    Touch t;
                    t.first = now;
                    uint64_t bodies[2] = {key.first, key.second};
                    bool you[2] = {swap ? s.human_b : s.human_a, swap ? s.human_a : s.human_b};
                    for (int i = 0; i < 2; i++) {
                        t.c[i].body = bodies[i], t.c[i].you = you[i];
                        t.c[i].ok = kizi::read(bodies[i] + 0x1c4, t.c[i].y0);
                    }
                    it = touches.emplace(key, t).first;
                }
                Touch &t = it->second;
                t.last = now, t.steps++;
                if (s.tyre)
                    t.tyre_steps++;
                t.ny_max = std::max(t.ny_max, s.ny_new), t.vn_max = std::max(t.vn_max, s.vn);
                t.c[swap ? 1 : 0].given += s.dvy_a, t.c[swap ? 0 : 1].given += s.dvy_b;
                if (body_left > 0 && !s.tyre && s.delta_v > 0) {
                    body_left--;
                    swofty::log(SIM_TAG " body %s/%s %s%s | flag32 %u ratio %.2f | normal.y %.2f | vn %.1f dv %.1f vt %.1f mu %.2f",
                                tag_text(s.tag_a).c_str(), tag_text(s.tag_b).c_str(), s.human_a ? "YOU" : "AI",
                                s.human_b ? "+YOU" : "+AI", s.flag32, s.ratio, s.ny_new, s.vn, s.delta_v, s.vt, s.mu_used);
                }
            }
            for (auto it = touches.begin(); it != touches.end();) {
                Touch &t = it->second;
                if (now - t.first <= 1500)
                    for (auto &w : t.c) {
                        float v[3], u[3];          // speed along the car's own up axis: a hill does not count, a lift does
                        if (w.ok && kizi::read(w.body + 0x220, v, 12) && kizi::read(w.body + 0x1a0, u, 12)) {
                            float ul = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
                            float up = ul > 0.5f ? (v[0] * u[0] + v[1] * u[1] + v[2] * u[2]) / ul : 0.0f;
                            w.vy_peak = std::max(w.vy_peak, up);
                            w.rise += std::max(0.0f, up) * 0.05f;
                        }
                    }
                if (now - t.first < 1500 || now - t.last < 300) {
                    ++it;
                    continue;
                }
                bool lifted = t.c[0].rise > 0.05f || t.c[1].rise > 0.05f;
                if (touch_left > 0 && (t.tyre_steps > 0 || lifted)) {
                    touch_left--;
                    auto car = [](const Watch &w) {
                        char b[160];
                        snprintf(b, sizeof b, "%s lifted %.2f m off its floor, peak up %.2f m/s, we gave %.2f m/s", w.you ? "YOU" : "AI", w.rise,
                                 w.vy_peak, w.given);
                        return std::string(b);
                    };
                    swofty::log(SIM_TAG " touch %s: %d record(s), %d tyre-on-tyre, max closing %.1f m/s, steepest normal.y %.2f | %s | %s",
                                t.tyre_steps ? "TYRE" : "body", t.steps, t.tyre_steps, t.vn_max, t.ny_max,
                                car(t.c[0]).c_str(), car(t.c[1]).c_str());
                }
                it = touches.erase(it);
            }
        } else {
            read_at = g_ring_next;
            touches.clear();
        }
        if (tick % 600 == 599) {
            body_left = 10, touch_left = 25;
            if (g_seen)
                swofty::log(SIM_TAG " rejected: no object %u, no vehicle %u, same %u, car A %u, car B %u, apply-to %u, geometry %u, "
                            "not flagged %u | last car values: inverse mass %g, body flags 0x%x, vehicle flag 0x%x",
                            g_rej[0].exchange(0), g_rej[1].exchange(0), g_rej[2].exchange(0), g_rej[3].exchange(0),
                            g_rej[4].exchange(0), g_rej[5].exchange(0), g_rej[6].exchange(0), g_rej[7].exchange(0),
                            double(g_dbg_invm.load()), g_dbg_flags.load(), g_dbg_vflag.load());
            if (g_seen)
                swofty::log(SIM_TAG " last 30 s: %u record(s), %u car-car, %u tyre-on-tyre, %u normal(s) rewritten, %u impulse(s) "
                            "applied, %u ignored, largest dv %.1f m/s, steepest normal.y %.2f, tyre touches with spin %u, largest vertical dv %.2f m/s, "
                            "capped (same step / paused) %u",
                            g_seen.exchange(0), g_carcar.exchange(0), g_tyre.exchange(0), g_rewritten.exchange(0),
                            g_applied.exchange(0), g_ignored.exchange(0), g_max_dv.exchange(0), g_max_ny.exchange(0),
                            g_spin_touches.exchange(0), double(g_max_lift.exchange(0)), g_step_capped.exchange(0));
        }
    }
}

// --- install -------------------------------------------------------------------------------------------------------------
bool install()
{
    uintptr_t hit = kizi::scan_code("48 39 0D ?? ?? ?? ?? 0F 84 ?? ?? ?? ?? 48 89 CE 48 89 0D ?? ?? ?? ?? 48 8B 41 48 48 8B 49 50 31 D2",
                                    "contact-sim: contact handler start");
    if (!hit)
        return false;
    int32_t d1 = 0, rel = 0;
    if (!kizi::read(hit + 3, d1) || !kizi::read(hit + 9, rel))
        return false;
    g_last_slot = hit + 7 + d1;
    uintptr_t epi = hit + 13 + rel;                // the handler's common exit (its first early-out jumps there too)
    const uint8_t want[6] = {0xC5, 0xF8, 0x28, 0x74, 0x24, 0x40};   // vmovaps xmm6,[rsp+40h]
    uint8_t orig[6] = {};
    if (!kizi::read(epi, orig, 6) || memcmp(orig, want, 6) != 0) {
        swofty::log(SIM_TAG " the handler's exit does not look as expected - not installed");
        return false;
    }
#if KIZI_SIM_HEAVY
    g_apply = reinterpret_cast<ApplyImpulseFn>(
        kizi::scan_code("56 48 83 EC 70 C5 78 29 54 24 60 C5 78 29 4C 24 50 C5 78 29 44 24 40 C5 F8 29 7C 24 30 C5 F8 29 74 "
                        "24 20 48 89 CE 8B 81 6C 04 00 00",
                        "contact-sim: apply impulse"));
    if (!g_apply) {
        swofty::log(SIM_TAG " the game's apply-impulse function was not found - not installed");
        return false;
    }
#endif
    uint8_t *m = kizi::alloc_near(epi, 4096);
    if (!m)
        return false;
    uint8_t *p = m;
    auto put32 = [&](int32_t v) { memcpy(p, &v, 4), p += 4; };
    auto put64 = [&](uint64_t v) { memcpy(p, &v, 8), p += 8; };
    for (uint8_t b : {0x55, 0x48, 0x89, 0xE5, 0x48, 0x83, 0xE4, 0xF0, 0x48, 0x81, 0xEC, 0x00, 0x01, 0x00, 0x00})
        *p++ = b;                                  // push rbp; mov rbp,rsp; and rsp,-16; sub rsp,100h
    const uint8_t save[6][5] = {{0x48, 0x89, 0x44, 0x24, 0x20}, {0x48, 0x89, 0x54, 0x24, 0x28}, {0x4C, 0x89, 0x44, 0x24, 0x30},
                                {0x4C, 0x89, 0x4C, 0x24, 0x38}, {0x4C, 0x89, 0x54, 0x24, 0x40}, {0x4C, 0x89, 0x5C, 0x24, 0x48}};
    for (auto &i : save)
        for (uint8_t b : i)
            *p++ = b;                              // rax, rdx, r8, r9, r10, r11 -> [rsp+20h..48h]
    for (int i = 0; i < 6; i++) {                  // xmm0-5 -> [rsp+50h+16i]
        *p++ = 0xC5, *p++ = 0xF8, *p++ = 0x29, *p++ = uint8_t(0x84 | (i << 3)), *p++ = 0x24;
        put32(0x50 + 16 * i);
    }
    for (uint8_t b : {0x48, 0x89, 0xF1})
        *p++ = b;                                  // mov rcx,rsi (the contact record)
    *p++ = 0x48, *p++ = 0xB8, put64(reinterpret_cast<uint64_t>(&on_contact));   // mov rax,on_contact
    *p++ = 0xFF, *p++ = 0xD0;                      // call rax
    for (int i = 0; i < 6; i++) {
        *p++ = 0xC5, *p++ = 0xF8, *p++ = 0x28, *p++ = uint8_t(0x84 | (i << 3)), *p++ = 0x24;
        put32(0x50 + 16 * i);
    }
    const uint8_t load[6][5] = {{0x48, 0x8B, 0x44, 0x24, 0x20}, {0x48, 0x8B, 0x54, 0x24, 0x28}, {0x4C, 0x8B, 0x44, 0x24, 0x30},
                                {0x4C, 0x8B, 0x4C, 0x24, 0x38}, {0x4C, 0x8B, 0x54, 0x24, 0x40}, {0x4C, 0x8B, 0x5C, 0x24, 0x48}};
    for (auto &i : load)
        for (uint8_t b : i)
            *p++ = b;
    *p++ = 0x48, *p++ = 0x89, *p++ = 0xEC, *p++ = 0x5D;   // mov rsp,rbp; pop rbp
    memcpy(p, orig, 6), p += 6;                    // the instruction we replaced
    *p++ = 0xFF, *p++ = 0x25, put32(0);            // jmp [rip+0] -> epi+6
    put64(epi + 6);
    FlushInstructionCache(GetCurrentProcess(), m, 4096);
    uint8_t jump[6] = {0xE9, 0, 0, 0, 0, 0x90};
    int32_t r = int32_t(reinterpret_cast<intptr_t>(m) - intptr_t(epi + 5));
    memcpy(jump + 1, &r, 4);
    return kizi::patch_code(epi, jump, 6);
}

}  // namespace sim

namespace sim {
// Install the hook and start the log worker. Standalone test addons call this from their own load; Contact Physics calls it
// from its load when embedded.
inline bool start()
{
    bool ok = install();
    swofty::log(SIM_TAG " contact hook %s | normal %.*s, apply to %.*s, log %s", ok ? "installed" : "NOT installed",
                (int)cfg::normal_mode().size(), cfg::normal_mode().data(), (int)cfg::apply_to().size(), cfg::apply_to().data(),
                cfg::contact_log() ? "on" : "off");
#if KIZI_SIM_HEAVY
    swofty::log(SIM_TAG " strength (exaggerate): %.*s", (int)cfg::exaggerate().size(), cfg::exaggerate().data());
    swofty::log(SIM_TAG " own response: restitution %.*s, friction %.*s, separation %.*s, engine response %.*s",
                (int)cfg::restitution().size(), cfg::restitution().data(), (int)cfg::friction().size(), cfg::friction().data(),
                (int)cfg::separation().size(), cfg::separation().data(), (int)cfg::engine_response().size(),
                cfg::engine_response().data());
#endif
    if (ok)
        std::thread(worker).detach();
    return ok;
}
}  // namespace sim

#ifndef KIZI_SIM_EMBED
SWOFTY_ON_LOAD
{
    sim::start();
}

SWOFTY_ADDON()
#endif
