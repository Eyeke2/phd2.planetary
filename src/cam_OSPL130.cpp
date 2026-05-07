/*
 *  cam_OSPL130.cpp
 *  PHD Guiding
 *
 *  Created by Craig Stark.
 *  Copyright (c) 2006, 2007, 2008, 2009, 2010 Craig Stark.
 *  All rights reserved.
 *
 */
#include "phd.h"
#include "camera.h"
#include "time.h"
#include "image_math.h"
#include <wx/stopwatch.h>

#if defined(OS_PL130)
# include "cam_OSPL130.h"
# include "cameras/OSPL130API.h"

static bool DLLExists(const wxString& DLLName)
{
    wxStandardPathsBase& StdPaths = wxStandardPaths::Get();
    if (wxFileExists(StdPaths.GetExecutablePath().BeforeLast(PATHSEPCH) + PATHSEPSTR + DLLName))
        return true;
    if (wxFileExists(StdPaths.GetExecutablePath().BeforeLast(PATHSEPCH) + PATHSEPSTR + ".." + PATHSEPSTR + DLLName))
        return true;
    if (wxFileExists(wxGetOSDirectory() + PATHSEPSTR + DLLName))
        return true;
    if (wxFileExists(wxGetOSDirectory() + PATHSEPSTR + "system32" + PATHSEPSTR + DLLName))
        return true;
    return false;
}

CameraOpticstarPL130::CameraOpticstarPL130()
{
    Connected = false;
    Name = _T("Opticstar PL-130M");
    FrameSize = wxSize(1280, 1024);
    m_hasGuideOutput = false;
    HasGainControl = false;
}

wxByte CameraOpticstarPL130::BitsPerPixel()
{
    return 16;
}

bool CameraOpticstarPL130::Connect(const wxString& camId)
{
    // returns true on error

    if (!DLLExists("OSPL130RT.dll"))
        return CamConnectFailed(_("Cannot find OSPL130RT.dll"));

    int retval = OSPL130_Initialize((int) HasBayer, false, 0, 2);
    if (retval)
        return CamConnectFailed(_("Cannot init camera"));

    // OSPL130_SetGain(6);
    Connected = true;
    return false;
}

bool CameraOpticstarPL130::Disconnect()
{
    OSPL130_Finalize();
    Connected = false;
    return false;
}

bool CameraOpticstarPL130::Capture(usImage& img, const CaptureParams& captureParams)
{
    int duration = captureParams.duration;
    int options = captureParams.captureOptions;

    bool still_going = true;

    int mode = 3 * (int) HasBayer;
    if (img.Init(FrameSize))
    {
        DisconnectWithAlert(CAPT_FAIL_MEMORY);
        return true;
    }
    if (OSPL130_Capture(mode, duration))
    {
        pFrame->Alert(_("Cannot start exposure"));
        return true;
    }
    if (duration > 100)
    {
        wxMilliSleep(duration - 100); // wait until near end of exposure, nicely
        // FIXME(ui-safety): wxApp::Yield() pumps the wx event loop reentrantly
        // and can deadlock against a remote-control client via wx's internal
        // wxFindWindowAtPoint() hit-test. See src/ui_safety.h and the
        // threading-rule comment block above handle_request() in
        // src/event_server.cpp. This site runs ONCE PER EXPOSURE on the
        // OSPL130 capture path, so the deadlock surface is large under any
        // workload that has a connected remote-control client. Real fix is
        // per-driver-thread for OSPL130 capture; UI_SAFE_YIELD() would
        // only LOG the violation, not remove it.
        wxGetApp().Yield();
        //      if (Abort) {
        //          MeadeCam->AbortImage();
        //          return true;
        //      }
    }
    while (still_going)
    { // wait for image to finish and d/l
        wxMilliSleep(20);
        OSPL130_IsExposing(&still_going);
        // FIXME(ui-safety): worst offender in this file -- wxApp::Yield()
        // here runs once per ~20 ms polling iteration during EVERY image
        // download. Same deadlock class as the per-exposure yield above;
        // see src/ui_safety.h for the full rationale.
        wxGetApp().Yield();
    }
    // Download
    OSPL130_GetRawImage(0, 0, FrameSize.GetWidth(), FrameSize.GetHeight(), (void *) img.ImageData);
    // byte swap

    if (options & CAPTURE_SUBTRACT_DARK)
        SubtractDark(img);
    if ((options & CAPTURE_RECON) && HasBayer && captureParams.CombinedBinning() == 1)
        QuickLRecon(img);

    return false;
}

#endif
