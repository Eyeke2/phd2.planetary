/*
 *  scope_oncamera.cpp
 *  PHD Guiding
 *
 *  Created by Bret McKee
 *  Copyright (c) 2012 Bret McKee
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
 *    Neither the name of Bret McKee, Dad Dog Development,
 *     Craig Stark, Stark Labs nor the names of its
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

#ifdef GUIDE_ONCAMERA

ScopeOnCamera::ScopeOnCamera() : m_tracking(true)
{
    m_Name = "On Camera";
}

ScopeOnCamera::~ScopeOnCamera() { }

// The simulated tracking flag is only meaningful when the connected camera is the internal
// simulator (which has no physical tracking, so this flag IS its source of truth). For real
// ST4-output cameras there is no tracking command on the ST4 wire, so exposing a flag that
// has no physical effect would be misleading.
static bool IsCameraSimulator()
{
    return pCamera && pCamera->Name == _T("Simulator");
}

bool ScopeOnCamera::Connect()
{
    // Start every session with tracking on, so the tracking gate doesn't fire on the first
    // pulse after reconnecting following a SetTracking(false) in a previous session.
    m_tracking = true;
    SetLastKnownTracking(true);
    return ScopeOnboardST4::ConnectOnboardST4(pCamera);
}

bool ScopeOnCamera::RequiresCamera()
{
    return true;
}

bool ScopeOnCamera::HasNonGuiMove()
{
    return true;
}

bool ScopeOnCamera::GetTracking(bool *tracking, bool verbose)
{
    if (!IsConnected() || !IsCameraSimulator())
        return Scope::GetTracking(tracking, verbose);
    *tracking = m_tracking;
    // Refresh the cached state for symmetry with ScopeASCOM::GetTracking; this keeps
    // Scope::IsKnownTrackingStopped() in sync without an extra round-trip.
    SetLastKnownTracking(m_tracking);
    if (verbose)
        Debug.Write(wxString::Format("ScopeOnCamera::GetTracking() = %d\n", m_tracking));
    return false;
}

bool ScopeOnCamera::SetTracking(bool tracking)
{
    if (!IsConnected() || !IsCameraSimulator())
        return Scope::SetTracking(tracking);
    m_tracking = tracking;
    // SetLastKnownTracking also clears the "Guiding stopped: the mount is not tracking."
    // alert on an off->on transition.
    SetLastKnownTracking(tracking);
    Debug.Write(wxString::Format("ScopeOnCamera::SetTracking(%d)\n", tracking));
    return false;
}

bool ScopeOnCamera::CanSetTracking()
{
    return IsCameraSimulator();
}

#endif // GUIDE_ONCAMERA
