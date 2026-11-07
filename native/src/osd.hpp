// SPDX-License-Identifier: MIT
// A short message in the middle of the screen (e.g. after a hotkey), drawn with Swofty's overlay.
// Swofty's overlay draws boxes and lines but not text yet (its text op is a no-op), so the text is a 5x7 pixel
// font made of small boxes. Needs the experimental "overlay" feature (swofty-features.txt on this install).
#pragma once
#include <windows.h>
#include <atomic>
#include <cstring>
#include <mutex>
#include <string>
#include <swofty/hud.hpp>
#include <swofty/overlay.hpp>

namespace osd {

// 5x7 glyphs, one byte per row, bit 4 = leftmost column.
struct Glyph {
    char c;
    uint8_t rows[7];
};
inline const Glyph FONT[] = {
    {'A', {14, 17, 17, 31, 17, 17, 17}}, {'B', {30, 17, 17, 30, 17, 17, 30}}, {'C', {14, 17, 16, 16, 16, 17, 14}},
    {'D', {30, 17, 17, 17, 17, 17, 30}}, {'E', {31, 16, 16, 30, 16, 16, 31}}, {'F', {31, 16, 16, 30, 16, 16, 16}},
    {'G', {14, 17, 16, 23, 17, 17, 15}}, {'H', {17, 17, 17, 31, 17, 17, 17}}, {'I', {14, 4, 4, 4, 4, 4, 14}},
    {'J', {7, 2, 2, 2, 2, 18, 12}},      {'K', {17, 18, 20, 24, 20, 18, 17}}, {'L', {16, 16, 16, 16, 16, 16, 31}},
    {'M', {17, 27, 21, 21, 17, 17, 17}}, {'N', {17, 17, 25, 21, 19, 17, 17}}, {'O', {14, 17, 17, 17, 17, 17, 14}},
    {'P', {30, 17, 17, 30, 16, 16, 16}}, {'Q', {14, 17, 17, 17, 21, 18, 13}}, {'R', {30, 17, 17, 30, 20, 18, 17}},
    {'S', {15, 16, 16, 14, 1, 1, 30}},   {'T', {31, 4, 4, 4, 4, 4, 4}},       {'U', {17, 17, 17, 17, 17, 17, 14}},
    {'V', {17, 17, 17, 17, 17, 10, 4}},  {'W', {17, 17, 17, 21, 21, 21, 10}}, {'X', {17, 17, 10, 4, 10, 17, 17}},
    {'Y', {17, 17, 10, 4, 4, 4, 4}},     {'Z', {31, 1, 2, 4, 8, 16, 31}},     {'0', {14, 17, 19, 21, 25, 17, 14}},
    {'1', {4, 12, 4, 4, 4, 4, 14}},      {'2', {14, 17, 1, 2, 4, 8, 31}},     {'3', {31, 2, 4, 2, 1, 17, 14}},
    {'4', {2, 6, 10, 18, 31, 2, 2}},     {'5', {31, 16, 30, 1, 1, 17, 14}},   {'6', {6, 8, 16, 30, 17, 17, 14}},
    {'7', {31, 1, 2, 4, 8, 8, 8}},       {'8', {14, 17, 17, 14, 17, 17, 14}}, {'9', {14, 17, 17, 15, 1, 2, 12}},
    {':', {0, 12, 12, 0, 12, 12, 0}},    {'+', {0, 4, 4, 31, 4, 4, 0}},       {'-', {0, 0, 0, 31, 0, 0, 0}},
    {'(', {2, 4, 8, 8, 8, 4, 2}},        {')', {8, 4, 2, 2, 2, 4, 8}},        {'.', {0, 0, 0, 0, 0, 12, 12}},
    {'/', {0, 1, 2, 4, 8, 16, 0}},       {'%', {24, 25, 2, 4, 8, 19, 3}},
};

inline const uint8_t *glyph(char c)
{
    if (c >= 'a' && c <= 'z')
        c = char(c - 32);
    for (auto &g : FONT)
        if (g.c == c)
            return g.rows;
    return nullptr;                                  // space and unknown characters: blank
}

inline std::mutex lock;
inline std::string line1, line2;
inline std::atomic<DWORD> until{0};
inline std::atomic<bool> enabled{true};

// Show two lines for `ms` milliseconds (line2 may be empty).
// Show two lines for `ms` milliseconds (line2 may be empty). With Swofty 1.5.3+ this is the race HUD's own
// notification (the game's banner for DRS enabled, a fastest lap or a penalty; `icon` one of its race flag icons, e.g.
// safety_car); otherwise, or if that is not available, our overlay box.
inline void show(const std::string &a, const std::string &b = "", DWORD ms = 2000, const char *icon = "")
{
    if (!enabled)
        return;
    if (swofty::hud::available() &&
        swofty::hud::toast(a, b, ms / 1000.0f < 2 ? 2.0f : ms / 1000.0f, icon) == swofty::hud::Result::ok)
        return;
    std::lock_guard<std::mutex> g(lock);
    line1 = a, line2 = b;
    until = GetTickCount() + ms;
}

inline void draw_text(SwoftyPaint *p, float x, float y, float px, uint32_t rgba, const std::string &s)
{
    for (char c : s) {
        if (const uint8_t *rows = glyph(c))
            for (int r = 0; r < 7; r++)
                for (int col = 0; col < 5; col++)
                    if (rows[r] & (16 >> col))
                        swofty::overlay::rect(p, x + col * px, y + r * px, px, px, rgba);
        x += 6 * px;
    }
}

inline void paint(SwoftyPaint *p, void *)
{
    if (!enabled || int(until.load() - GetTickCount()) <= 0)
        return;
    std::string a, b;
    {
        std::lock_guard<std::mutex> g(lock);
        a = line1, b = line2;
    }
    float w = 0, h = 0;
    swofty::overlay::size(p, &w, &h);
    if (w < 100 || h < 100)
        return;
    float px = h / 160 > 3 ? h / 160 : 3;            // 6.75 px at 1080p
    float px2 = px * 0.7f;
    float wa = a.size() * 6 * px, wb = b.size() * 6 * px2;
    float bw = (wa > wb ? wa : wb) + 8 * px, bh = 7 * px + (b.empty() ? 0 : 9 * px2 + 2 * px) + 6 * px;
    float bx = (w - bw) / 2, by = h * 0.32f - bh / 2;
    swofty::overlay::rect(p, bx, by, bw, bh, 0x000000c0);
    swofty::overlay::rect_outline(p, bx, by, bw, bh, px / 2, 0xe10600ffu);
    draw_text(p, (w - wa) / 2, by + 3 * px, px, 0xffffffffu, a);
    if (!b.empty())
        draw_text(p, (w - wb) / 2, by + 3 * px + 7 * px + 2 * px, px2, 0xc8c8c8ffu, b);
}

// Call from SWOFTY_ON_LOAD. Returns false when the overlay is not available (feature off, older core).
inline bool start()
{
    if (!swofty::overlay::available())
        return false;
    return swofty::overlay::on_paint(paint) == swofty::overlay::Result::ok;
}

}  // namespace osd
