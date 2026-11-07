// SPDX-License-Identifier: MIT
// Small helpers shared by the Kizi addons: code scans that work on more than one F1 25 build, crash-safe memory
// reads and writes, a heap search, and a jump stub placed within rel32 reach of the game code.
#pragma once
#include <windows.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>
#include <swofty/mixin.hpp>

namespace kizi {

// The official F1 25 build keeps its code in .xtls; other builds (e.g. the one we test on) keep it in .bss.
// swofty's scan takes a section name and needs exactly one hit, so try each name until one is unique.
inline uintptr_t scan_code(const char *pattern, const char *what)
{
    static const char *const sections[] = {".xtls", ".bss", ".text"};
    for (const char *s : sections) {
        uintptr_t hit = 0;
        int n = swofty::api->scan(s, pattern, &hit);
        if (n == 1)
            return hit;
        if (n > 1) {
            swofty::log("[kizi] %s: %d matches in %s, not unique - skipped", what, n, s);
            return 0;
        }
    }
    swofty::log("[kizi] %s: code pattern not found (game update?)", what);
    return 0;
}

inline uintptr_t image_base() { return swofty::api->module_base(); }

inline size_t image_size()
{
    auto base = reinterpret_cast<const uint8_t *>(image_base());
    auto dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(base);
    auto nt = reinterpret_cast<const IMAGE_NT_HEADERS *>(base + dos->e_lfanew);
    return nt->OptionalHeader.SizeOfImage;
}

inline bool in_image(uintptr_t a) { return a - image_base() < image_size(); }

// ReadProcessMemory on our own process: returns false instead of crashing when the game frees the memory meanwhile.
inline bool read(uintptr_t a, void *out, size_t n)
{
    SIZE_T got = 0;
    return ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<void *>(a), out, n, &got) && got == n;
}

template <class T> inline bool read(uintptr_t a, T &out) { return read(a, &out, sizeof out); }

inline bool write(uintptr_t a, const void *data, size_t n)
{
    SIZE_T put = 0;
    return WriteProcessMemory(GetCurrentProcess(), reinterpret_cast<void *>(a), data, n, &put) && put == n;
}

// Patch executable bytes (temporarily writable, then the old protection and an instruction-cache flush).
inline bool patch_code(uintptr_t a, const void *data, size_t n)
{
    DWORD old = 0, tmp = 0;
    if (!VirtualProtect(reinterpret_cast<void *>(a), n, PAGE_EXECUTE_READWRITE, &old))
        return false;
    memcpy(reinterpret_cast<void *>(a), data, n);
    VirtualProtect(reinterpret_cast<void *>(a), n, old, &tmp);
    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void *>(a), n);
    return true;
}

// Visit every committed, private, read-write region (the game's heap) in chunks copied into a local buffer.
// fn(chunk_base, bytes, size) returns false to stop.
template <class F> inline void each_heap_chunk(F fn)
{
    std::vector<uint8_t> buf(1 << 20);
    uintptr_t a = 0x10000;
    MEMORY_BASIC_INFORMATION m;
    while (a < 0x7FFFFFFF0000ull && VirtualQuery(reinterpret_cast<void *>(a), &m, sizeof m)) {
        uintptr_t start = reinterpret_cast<uintptr_t>(m.BaseAddress), end = start + m.RegionSize;
        if (m.State == MEM_COMMIT && m.Type == MEM_PRIVATE && m.Protect == PAGE_READWRITE) {
            for (uintptr_t c = start; c < end; c += buf.size()) {
                size_t n = std::min<uintptr_t>(buf.size(), end - c);
                if (read(c, buf.data(), n) && !fn(c, buf.data(), n))
                    return;
            }
        }
        a = end;
    }
}

// Allocate a small executable block within +-2 GB of `target_addr` (so a rel32 call can reach it).
inline uint8_t *alloc_near(uintptr_t target_addr, size_t size)
{
    const uintptr_t gran = 0x10000, reach = 0x7FF00000;
    for (int dir = -1; dir <= 1; dir += 2) {
        uintptr_t a = dir < 0 ? (image_base() & ~(gran - 1)) - gran
                              : ((image_base() + image_size() + gran - 1) & ~(gran - 1));
        while ((dir < 0 ? target_addr - a : a - target_addr) < reach && a > gran) {
            MEMORY_BASIC_INFORMATION m;
            if (!VirtualQuery(reinterpret_cast<void *>(a), &m, sizeof m))
                break;
            if (m.State == MEM_FREE) {
                void *p = VirtualAlloc(reinterpret_cast<void *>(a), size, MEM_RESERVE | MEM_COMMIT,
                                       PAGE_EXECUTE_READWRITE);
                if (p)
                    return static_cast<uint8_t *>(p);
            }
            a = dir < 0 ? a - gran : a + gran;
        }
    }
    return nullptr;
}

// Point the `call rel32` at `site` to `target` through a 14-byte absolute-jump stub. Returns the old callee, or 0.
inline uintptr_t redirect_call(uintptr_t site, void *target)
{
    uint8_t op = 0;
    int32_t rel = 0;
    if (!read(site, op) || op != 0xE8 || !read(site + 1, rel))
        return 0;
    uintptr_t old = site + 5 + rel;
    uint8_t *stub = alloc_near(site, 0x1000);
    if (!stub)
        return 0;
    const uint8_t jmp[] = {0xFF, 0x25, 0, 0, 0, 0};              // jmp qword [rip+0]
    memcpy(stub, jmp, sizeof jmp);
    uint64_t abs = reinterpret_cast<uint64_t>(target);
    memcpy(stub + sizeof jmp, &abs, 8);
    FlushInstructionCache(GetCurrentProcess(), stub, 14);
    int32_t nrel = static_cast<int32_t>(reinterpret_cast<uintptr_t>(stub) - (site + 5));
    return patch_code(site + 1, &nrel, 4) ? old : 0;
}

}  // namespace kizi
