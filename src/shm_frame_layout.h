/*
 *  shm_frame_layout.h
 *  PHD Guiding
 *
 *  Created by Leo Shatz.
 *  Copyright (c) 2026 Leo Shatz
 *  All rights reserved.
 *
 *  This source code is distributed under the following "BSD" license
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *    Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *    Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *    Neither the name of Craig Stark, Stark Labs nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>

// Shared-memory frame layout. Both ends must use identical copies of this file.

#define SHM_FRAME_MAGIC      0x314D5346u
#define SHM_FRAME_VERSION    3u
#define SHM_FRAME_SLOTS      2
// 20 MiB per slot: holds a 16-bit frame up to ~10.4 Mpixel (e.g. 3008x3008).
// Larger sensors are decimated by PHD2 before publishing.
#define SHM_FRAME_SLOT_BYTES ((size_t) 20 * 1024 * 1024)

#pragma pack(push, 4)
struct ShmFrameMeta
{
    uint32_t dataLength;
    uint16_t width;        // buffer (possibly decimated) dimensions
    uint16_t height;
    uint16_t origWidth;    // full-frame dimensions before decimation
    uint16_t origHeight;
    uint16_t binning;
    uint16_t exposureMs;
    uint16_t bitsPerPixel;
    uint16_t reserved;
    uint32_t pixelSize;
    uint32_t frameCounter;
    double   timestamp;
};
struct ShmFrameLayout
{
    uint32_t magic;
    uint32_t version;
    uint32_t slotBytes;
    uint32_t slotCount;
    volatile int32_t publishedSlot;
    volatile int32_t seq[SHM_FRAME_SLOTS];
    ShmFrameMeta     meta[SHM_FRAME_SLOTS];
};
#pragma pack(pop)

static constexpr size_t SHM_FRAME_PIXELS_OFFSET = (sizeof(ShmFrameLayout) + 63) & ~((size_t) 63);
