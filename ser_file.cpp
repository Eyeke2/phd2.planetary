/*
 *  ser_file.cpp
 *  PHD Guiding
 *
 *  Planetary detection extensions by Leo Shatz
 *  Copyright (c) 2023-2024 Leo Shatz
 *  Copyright (C) 2015 Chris Garry
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

#include "phd.h"
#include "guider_planetary.h"
#include "planetary_tool.h"
#include "ser_file.h"

#include <numeric>

SERFile::SERFile()
{
    memset(&m_serHeader, 0, sizeof(m_serHeader));
    m_serFile = nullptr;
    m_serFileName = wxEmptyString;
    m_framesWritten = 0;
}

SERFile::~SERFile()
{
    if (m_serFile)
        Close();
}

SERFile::SERFile(const wxString& fileName, int width, int height, wxString observerName, wxString instrumentName, wxString telescopeName)
{
    m_serFile = nullptr;
    m_serFileName = fileName;
    m_framesWritten = 0;

    // Get the current date and time
    wxDateTime now = wxDateTime::UNow();
    uint64_t localTimeStamp = DateToTimestamp(now.GetYear(), now.GetMonth(), now.GetDay(), now.GetHour(), now.GetMinute(), now.GetSecond(), now.GetMillisecond() * 1000);
    wxDateTime utc = now.ToUTC();
    uint64_t utcTimeStamp = DateToTimestamp(utc.GetYear(), utc.GetMonth(), utc.GetDay(), utc.GetHour(), utc.GetMinute(), utc.GetSecond(), utc.GetMillisecond() * 1000);

    // Initialize the SER header
    memset(&m_serHeader, 0, sizeof(m_serHeader));
    strncpy(m_serHeader.FileID, "LUCAM-RECORDER", sizeof(m_serHeader.FileID));
    m_serHeader.LuID = 0;         // Unknown
    m_serHeader.ColorID = 0;      // Monochrome
    m_serHeader.LittleEndian = 1; // Intel
    m_serHeader.ImageWidth = width;
    m_serHeader.ImageHeight = height;
    m_serHeader.PixelDepth = 8;
    m_serHeader.FrameCount = 0;
    strncpy(m_serHeader.Observer, observerName.ToStdString().c_str(), 40);
    strncpy(m_serHeader.Instrument, instrumentName.ToStdString().c_str(), 40);
    strncpy(m_serHeader.Telescope, telescopeName.ToStdString().c_str(), 40);
    m_serHeader.DateTime_UTC = utcTimeStamp;
    m_serHeader.DateTime = localTimeStamp;
}

bool SERFile::Open()
{
    if (!m_serFile)
    {
        const int BUFFER_SIZE = 64 * 1024;
        m_serFile = fopen(m_serFileName, "wb");
        m_fileBuffer = (char*)malloc(BUFFER_SIZE);
        if (m_fileBuffer)
            setvbuf(m_serFile, m_fileBuffer, _IOFBF, BUFFER_SIZE);
        m_timestamps.reserve(4096);
        Debug.Write(wxString::Format("Start video log file %s\n", m_serFileName));
    }
    if (m_serFile)
    {
        int records = fwrite(&m_serHeader, sizeof(SERHeader), 1, m_serFile);
        m_framesWritten = 0;
        return records == 1;
    }
    return false;
}

void SERFile::Close()
{
    if (m_serFile)
    {
        fwrite(m_timestamps.data(), sizeof(uint64_t), m_timestamps.size(), m_serFile);
        fseek(m_serFile, offsetof(SERHeader, FrameCount), SEEK_SET);
        fwrite(&m_framesWritten, sizeof(int), 1, m_serFile);
        fclose(m_serFile);
        free(m_fileBuffer);
        m_serFile = nullptr;
        Debug.Write(wxString::Format("Close video log file %s\n", m_serFileName));
    }
}

bool SERFile::WriteFrame(const cv::Mat& image)
{
    if (m_serFile)
    {
        uint64_t ts = GetCurrentLocalTimestamp();
        m_timestamps.push_back(ts);
        fwrite(image.data, image.elemSize() * image.total(), 1, m_serFile);
        m_framesWritten++;
        Debug.Write(wxString::Format("Added image frame in %s\n", m_serFileName));
        return true;
    }
    return false;
}

// Get the local timestamp
uint64_t SERFile::GetCurrentLocalTimestamp()
{
    wxDateTime now = wxDateTime::UNow();
    return DateToTimestamp(now.GetYear(), now.GetMonth(), now.GetDay(), now.GetHour(), now.GetMinute(), now.GetSecond(), now.GetMillisecond() * 1000);
}

// Get the UTC timestamp
uint64_t SERFile::GetCurrentUtcTimestamp()
{
    wxDateTime now = wxDateTime::UNow();
    wxDateTime utc = now.ToUTC();
    return DateToTimestamp(utc.GetYear(), utc.GetMonth(), utc.GetDay(), utc.GetHour(), utc.GetMinute(), utc.GetSecond(), utc.GetMillisecond() * 1000);
}

// ---------------------------------------------------------------------
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>
// ---------------------------------------------------------------------

// Calculate if a year is a leap yer
bool SERFile::IsLeapYear(uint32_t year)
{
    if ((year % 400) == 0) {
        // If year is divisible by 400 then is_leap_year
        return true;
    }
    else if ((year % 100) == 0) {
        // Else if year is divisible by 100 then not_leap_year
        return false;
    }
    else if ((year % 4) == 0) {
        // Else if year is divisible by 4 then is_leap_year
        return true;
    }
    else {
        // Else not_leap_year
        return false;
    }
}

// Convert date to SER timestamp
uint64_t SERFile::DateToTimestamp(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, int32_t second, int32_t microsec)
{
    uint64_t ts = 0;
    int32_t yr;

    // Add 400 year blocks
    for (yr = 1; yr < (year - 400); yr += 400)
        ts += m_septaseconds_per_400_years;

    // Add 1 years
    for (; yr < year; yr++)
    {
        uint32_t days_this_year = 365;
        if (IsLeapYear(yr))
            days_this_year = 366;

        ts += (days_this_year * m_septaseconds_per_day);
    }

    // Add months
    for (int mon = 1; mon < month; mon++)
    {
        switch (mon)
        {
        case 4:   // April
        case 6:   // June
        case 9:   // September
        case 11:  // Novenber
            ts += (30 * m_septaseconds_per_day);
            break;
        case 2:  // Feburary
            if (IsLeapYear(year))
                ts += (29 * m_septaseconds_per_day);
            else
                ts += (28 * m_septaseconds_per_day);

            break;
        default:
            ts += (31 * m_septaseconds_per_day);
            break;
        }
    }

    // Add days
    ts += ((day - 1) * m_septaseconds_per_day);

    // Add hours
    ts += (hour * m_septaseconds_per_hour);

    // Add minutes
    ts += (minute * m_septaseconds_per_minute);

    // Add seconds
    ts += (second * C_SEPASECONDS_PER_SECOND);

    // Micro seconds
    ts += (microsec * m_sepaseconds_per_microsecond);

    // Output result
    return ts;
}