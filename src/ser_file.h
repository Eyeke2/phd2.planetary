/*
 *  ser_file.h
 *  PHD Guiding
 *
 *  Planetary detection extensions by Leo Shatz
 *  Copyright (c) 2023-2024 Leo Shatz
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

#pragma pack(1)
struct SERHeader
{
    char FileID[14]; // "LUCAM-RECORDER"
    uint32_t LuID; // Camera identifier, can be set to 0 if unknown
    uint32_t ColorID; // Color format, 0 for monochrome
    uint32_t LittleEndian; // Byte order, 1 for little endian
    uint32_t ImageWidth; // Width of the image
    uint32_t ImageHeight; // Height of the image
    uint32_t PixelDepth; // Depth of each pixel in bits, 16 for 16-bit
    uint32_t FrameCount; // Total number of frames
    char Observer[40]; // Name of the observer, can be left blank
    char Instrument[40]; // Name of the instrument, can be left blank
    char Telescope[40]; // Name of the telescope, can be left blank
    uint64_t DateTime; // Date and time of the first frame in UTC, as a 64-bit integer
    uint64_t DateTime_UTC; // Same as DateTime, but if the local time is used in DateTime, this is UTC
};
#pragma pack()

// Implement SER file Open and AppendFrame methods
class SERFile
{
    char *m_fileBuffer;
    FILE *m_serFile;
    wxString m_serFileName;
    struct SERHeader m_serHeader;
    int m_framesWritten;

private:
    static const uint64_t C_SEPASECONDS_PER_SECOND = 10000000;
    static const uint64_t m_sepaseconds_per_microsecond = 10;
    static const uint64_t m_septaseconds_per_part_minute = C_SEPASECONDS_PER_SECOND * 6;
    static const uint64_t m_septaseconds_per_minute = C_SEPASECONDS_PER_SECOND * 60;
    static const uint64_t m_septaseconds_per_hour = C_SEPASECONDS_PER_SECOND * 60 * 60;
    static const uint64_t m_septaseconds_per_day = m_septaseconds_per_hour * 24;
    static const uint32_t m_days_in_400_years = 303 * 365 + 97 * 366;
    static const uint64_t m_septaseconds_per_400_years = m_days_in_400_years * m_septaseconds_per_day;

    std::vector<uint64_t> m_timestamps;

public:
    SERFile();
    SERFile(const wxString& fileName, int width, int height, wxString observerName, wxString instrumentName,
            wxString telescopeName);
    ~SERFile();

    bool Open();
    void Close();
    bool IsOpen() { return m_serFile != nullptr; }
    int FrameWidth() { return m_serHeader.ImageWidth; }
    int FrameHeight() { return m_serHeader.ImageHeight; }
    bool WriteFrame(const cv::Mat& image);
    bool IsLeapYear(uint32_t year);
    uint64_t GetCurrentUtcTimestamp();
    uint64_t GetCurrentLocalTimestamp();
    uint64_t DateToTimestamp(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, int32_t second,
                             int32_t microsec);
};
