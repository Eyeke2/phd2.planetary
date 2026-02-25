/*
 *  cam_FrameMon.h
 *  PHD Guiding
 *
 *  Created by Leo Shatz.
 *  Copyright (c) 2024-2025 Leo Shatz
 *  Copyright (c) 2018-2025 openphdguiding.org
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

struct frameDesc
{
public:
    frameDesc()
    {
        pixelSize = GuideCamera::UnknownPixelSize;
        pos = PHD_Point(0, 0);
        pos.Invalidate();
        quality = 0.f;
        sharpness = 0.f;
        snr = 0.f;
        mass = 0.f;
        peak = 0;
        radius = 0;
        minRadius = 0;
        maxRadius = 0;
        features = 0;
        time = 0.f;
    };

    double pixelSize;
    PHD_Point pos;
    float dispersion;
    float quality;
    float sharpness;
    float snr;
    float mass;
    float time;
    int peak;
    int radius;
    int minRadius;
    int maxRadius;
    int features;
};

class ImageFrameServer;

class CameraFrameMonitor : public GuideCamera
{
    void StartImageServer();

public:
    CameraFrameMonitor();
    ~CameraFrameMonitor();

    bool Capture(usImage& img, const CaptureParams& captureParams) override;
    bool Connect(const wxString& camId) override;
    bool Disconnect() override;
    bool HasNonGuiCapture() override { return true; }
    bool ST4HasNonGuiMove() override { return true; }
    bool ST4PulseGuideScope(int direction, int duration) override;
    wxByte BitsPerPixel() override;
    void InitCapture() override;
    bool GetCaptureDescriptor(void* desc) override;
    void SetProperty(const wxString prop, wxString value) override;
    void SetProperty(const wxString prop, int value) override;
    wxString GetStrProperty(const wxString prop, int timeout = 0) override;
    wxSize DarkFrameSize() override { return UNDEFINED_FRAME_SIZE; };

private:
    wxMutex m_lock;
    wxCondition m_sync;
    bool m_ready;
    wxString m_path;
    wxString m_physName;
    uint16_t m_imgPort;

    wxString m_cameraConnectAlertMsg;
    wxSize m_frameSize;
    wxString m_lastKnownGoodFilename;
    cv::Mat m_lastKnownImage;
    frameDesc m_imgDesc;

    wxMutex m_serverMutex;
    wxCondition m_serverCond;
    int m_useCount;
    ImageFrameServer *m_imageServer;
    bool GetServer();
    void PutServer();
};
