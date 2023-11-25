/*
 *  planetary_tool.cpp
 *  PHD Guiding

 *  Created by Leo Shatz.
 *  Copyright (c) 2023 openphdguiding.org
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
#include "planetary_tool.h"

#include <wx/valnum.h>

struct PlanetToolWin : public wxDialog
{

    PlanetToolWin();
    ~PlanetToolWin();

    void OnClose(wxCloseEvent& event);
    void UpdateStatus();
};

static wxString TITLE = wxTRANSLATE("Star tracking mode");
static wxString TITLE_ACTIVE = wxTRANSLATE("Planet tracking mode");

PlanetToolWin::PlanetToolWin()
: wxDialog(pFrame, wxID_ANY, wxGetTranslation(TITLE), wxDefaultPosition, wxDefaultSize, wxCAPTION | wxCLOSE_BOX)
{
    SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxCommandEvent notify;
}

PlanetToolWin::~PlanetToolWin(void)
{
    pFrame->pPlanetTool = 0;
}

void PlanetToolWin::UpdateStatus()
{
}

void PlanetToolWin::OnClose(wxCloseEvent& evt)
{
    Destroy();
}

wxWindow *PlanetTool::CreatePlanetToolWindow()
{
    return new PlanetToolWin();
}

void PlanetTool::NotifyUpdateLockPos()
{
    if (pFrame && pFrame->pPlanetTool)
    {
        PlanetToolWin *win = static_cast<PlanetToolWin *>(pFrame->pPlanetTool);
    }
}

void PlanetTool::UpdatePlanetToolControls(bool update)
{
    // notify planet tool to update its controls
    if (pFrame && pFrame->pPlanetTool)
    {
        wxCommandEvent event(APPSTATE_NOTIFY_EVENT, pFrame->GetId());
        event.SetEventObject(pFrame);
        event.SetInt(update ? 1 : 0);
        wxPostEvent(pFrame->pPlanetTool, event);
    }
}
