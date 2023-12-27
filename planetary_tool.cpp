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

#if FILE_SIMULATOR_MODE
wxString simulatorFileTemplate = _("");
int SimFileIndex = 1;
#endif

struct PlanetToolWin : public wxDialog
{
    wxToggleButton *m_enableButton;
    wxTextCtrl *m_status;

    wxStaticText     *m_minDist_Label;
    wxStaticText     *m_param1_Label;
    wxStaticText     *m_param2_Label;
    wxStaticText     *m_minRadius_Label;
    wxStaticText     *m_maxRadius_Label;
    wxStaticText     *m_BlockSize_Label;

    wxSpinCtrlDouble *m_minDist;
    wxSpinCtrlDouble *m_param1;
    wxSpinCtrlDouble *m_param2;
    wxSpinCtrlDouble *m_minRadius;
    wxSpinCtrlDouble *m_maxRadius;

    wxSlider         *m_ThresholdSlider;

    wxButton   *m_CloseButton;
    wxCheckBox *m_EclipseModeCheckBox;
    wxCheckBox* m_RoiCheckBox;
    wxCheckBox* m_ShowElements;
    bool        m_MouseHoverFlag;

    bool init_once;

    PlanetToolWin();
    ~PlanetToolWin();

    void OnClose(wxCloseEvent& event);
    void OnCloseButton(wxCommandEvent& event);
    void OnKeyDown(wxKeyEvent& event);
    void OnKeyUp(wxKeyEvent& event);
    void OnMouseEnterCloseBtn(wxMouseEvent& event);
    void OnMouseLeaveCloseBtn(wxMouseEvent& event);
    void OnThresholdChanged(wxCommandEvent& event);

    void OnAppStateNotify(wxCommandEvent& event);
    void OnEnableToggled(wxCommandEvent& event);
    void OnSpinCtrl_minDist(wxSpinDoubleEvent& event);
    void OnSpinCtrl_param1(wxSpinDoubleEvent& event);
    void OnSpinCtrl_param2(wxSpinDoubleEvent& event);
    void OnSpinCtrl_minRadius(wxSpinDoubleEvent& event);
    void OnSpinCtrl_maxRadius(wxSpinDoubleEvent& event);
    void OnEclipseModeClick(wxCommandEvent& event);
    void OnRoiModeClick(wxCommandEvent& event);
    void OnShowElementsClick(wxCommandEvent& event);

#if FILE_SIMULATOR_MODE
    wxTextCtrl* m_filePathTextCtrl;
    wxSpinCtrlDouble* m_fileIndex;
    void OnBrowseFileName(wxCommandEvent& event);
    void OnFileTextChange(wxCommandEvent& event);
    void OnSpinCtrl_FileIndex(wxSpinDoubleEvent& event);
#endif

    void UpdateStatus();
};

static wxString TITLE = wxTRANSLATE("Planetary tracking | disabled");
static wxString TITLE_ACTIVE = wxTRANSLATE("Planetary tracking | enabled");

static void SetEnabledState(PlanetToolWin* win, bool active)
{
    if (active)
    {
        win->SetTitle(wxGetTranslation(TITLE_ACTIVE));
        win->m_enableButton->SetLabel(_("Disable planet detection"));
    }
    else
    {
        win->SetTitle(wxGetTranslation(TITLE));
        win->m_enableButton->SetLabel(_("Enable planet detection"));
    }
    win->UpdateStatus();
}

PlanetToolWin::PlanetToolWin()
: wxDialog(pFrame, wxID_ANY, wxGetTranslation(TITLE), wxDefaultPosition, wxDefaultSize, wxCAPTION | wxCLOSE_BOX)
{
    SetSizeHints(wxDefaultSize, wxDefaultSize);

    m_minDist_Label = new wxStaticText(this, wxID_ANY, _("min dist:"));
    m_minDist_Label->Wrap(-1);
    m_param1_Label = new wxStaticText(this, wxID_ANY, _("param1:"));
    m_param1_Label->Wrap(-1);
    m_param2_Label = new wxStaticText(this, wxID_ANY, _("param2:"));
    m_param2_Label->Wrap(-1);
    m_minRadius_Label = new wxStaticText(this, wxID_ANY, _("min radius:"));
    m_minRadius_Label->Wrap(-1);
    m_maxRadius_Label = new wxStaticText(this, wxID_ANY, _("max radius:"));
    m_maxRadius_Label->Wrap(-1);

    m_minDist = new wxSpinCtrlDouble(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 1024, PT_MIN_DIST_DEFAULT);
    m_minDist_Label->SetToolTip(_("minimum distance between the centers of the detected circles"));

    m_param1 = new wxSpinCtrlDouble(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 255, PT_PARAM1_DEFAULT);
    m_param1_Label->SetToolTip(_("The higher threshold for the Canny edge detector. Increase this value to avoid false circles"));

    m_param2 = new wxSpinCtrlDouble(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 512, PT_PARAM2_DEFAULT);
    m_param2_Label->SetToolTip(_("The accumulator threshold for circle centers. Smaller values will mean more circle candidates, and larger values will suppress weaker circles. You might want to increase this value if you're getting false circles"));

    m_minRadius = new wxSpinCtrlDouble(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 1024, PT_MIN_RADIUS_DEFAULT);
    m_minRadius_Label->SetToolTip(_("Minimum planet radius in pixels. If set to 0, the minimal size is not limited."));

    m_maxRadius = new wxSpinCtrlDouble(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 1024, PT_MAX_RADIUS_DEFAULT);
    m_maxRadius_Label->SetToolTip(_("Maximum planet radius in pixels. If set to 0, the maximal size is not limited. If neither minRadius nor maxRadius is set, they are estimated from the image size."));

    m_enableButton = new wxToggleButton(this, wxID_ANY, TITLE, wxDefaultPosition, wxDefaultSize, 0);

    long style = wxSTATIC_BORDER | wxTE_MULTILINE;
    m_status = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(400, 80), style);
    m_status->Enable(false);

    wxBoxSizer *x_minDist = new wxBoxSizer(wxHORIZONTAL);
    x_minDist->Add(0, 0, 1, wxEXPAND, 5);
    x_minDist->Add(m_minDist_Label, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_minDist->Add(m_minDist, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_minDist->Add(0, 0, 1, wxEXPAND, 5);

    wxBoxSizer *x_param1 = new wxBoxSizer(wxHORIZONTAL);
    x_param1->Add(0, 0, 1, wxEXPAND, 5);
    x_param1->Add(m_param1_Label, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_param1->Add(m_param1, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_param1->Add(0, 0, 1, wxEXPAND, 5);

    wxBoxSizer *x_param2 = new wxBoxSizer(wxHORIZONTAL);
    x_param2->Add(0, 0, 1, wxEXPAND, 5);
    x_param2->Add(m_param2_Label, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_param2->Add(m_param2, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_param2->Add(0, 0, 1, wxEXPAND, 5);

    wxBoxSizer *x_minRadius = new wxBoxSizer(wxHORIZONTAL);
    x_minRadius->Add(0, 0, 1, wxEXPAND, 5);
    x_minRadius->Add(m_minRadius_Label, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_minRadius->Add(m_minRadius, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_minRadius->Add(0, 0, 1, wxEXPAND, 5);

    wxBoxSizer *x_maxRadius = new wxBoxSizer(wxHORIZONTAL);
    x_maxRadius->Add(0, 0, 1, wxEXPAND, 5);
    x_maxRadius->Add(m_maxRadius_Label, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_maxRadius->Add(m_maxRadius, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_maxRadius->Add(0, 0, 1, wxEXPAND, 5);

    wxStaticBoxSizer *ParamsSizer = new wxStaticBoxSizer(new wxStaticBox(this, wxID_ANY, _("Planet detection parameters")), wxVERTICAL);
    ParamsSizer->Add(x_minDist, 0, wxEXPAND, 5);
    ParamsSizer->Add(x_param1, 0, wxEXPAND, 5);
    ParamsSizer->Add(x_param2, 0, wxEXPAND, 5);
    ParamsSizer->Add(x_minRadius, 0, wxEXPAND, 5);
    ParamsSizer->Add(x_maxRadius, 0, wxEXPAND, 5);
    ParamsSizer->AddSpacer(10);

    // Eclipse mode stuff
    wxStaticText* ThresholdLabel = new wxStaticText(this, wxID_ANY, wxT("Edge Detection Threshold:"), wxDefaultPosition, wxDefaultSize, 0);
    m_ThresholdSlider = new wxSlider(this, wxID_ANY, 255, 0, 400, wxPoint(20, 20), wxSize(400, -1), wxSL_HORIZONTAL | wxSL_LABELS);
    ThresholdLabel->SetToolTip(_("Higher values reduce sensitivity to weaker edges, providing cleaner edge maps. Detected edges are shown in red."));
    m_ThresholdSlider->Bind(wxEVT_SLIDER, &PlanetToolWin::OnThresholdChanged, this);

    m_EclipseModeCheckBox = new wxCheckBox(this, wxID_ANY, _("Enable Eclipse mode"));
    m_EclipseModeCheckBox->SetToolTip(_("Enable Eclipse mode for enhanced tracking of partial solar / lunar discs"));
    m_RoiCheckBox = new wxCheckBox(this, wxID_ANY, _("Use ROI"));
    m_RoiCheckBox->SetToolTip(_("Enable ROI for improved processing speed and reduced CPU usage."));

    // Show/hide detected elements
    m_ShowElements = new wxCheckBox(this, wxID_ANY, _("Display internal edges/circles"));
    m_ShowElements->SetToolTip(_("Toggle the visibility of internally detected edges/circles and tune detection parameters to maintain a manageable number of these features while keeping them as close as possible to the light disk boundary."));

    // Close button
    m_MouseHoverFlag = false;
    wxBoxSizer *ButtonSizer = new wxBoxSizer(wxHORIZONTAL);
    m_CloseButton = new wxButton(this, wxID_ANY, _("Close"));
    ButtonSizer->Add(m_enableButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    ButtonSizer->AddSpacer(10);
    ButtonSizer->Add(m_CloseButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

#if FILE_SIMULATOR_MODE
    wxStaticText* fileFormatLabel = new wxStaticText(this, wxID_ANY, wxT("file path"));
    fileFormatLabel->Wrap(-1);
    wxStaticText* fileIndexLabel = new wxStaticText(this, wxID_ANY, wxT("index"));
    fileIndexLabel->Wrap(-1);

    m_filePathTextCtrl = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(200, -1));
    wxButton* browseButton = new wxButton(this, wxID_ANY, wxT("Browse"));

    m_fileIndex = new wxSpinCtrlDouble(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 9999, 1);

    wxBoxSizer* x_FilePath = new wxBoxSizer(wxHORIZONTAL);
    x_FilePath->Add(0, 0, 1, wxEXPAND, 5);
    x_FilePath->Add(fileFormatLabel, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_FilePath->Add(m_filePathTextCtrl, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_FilePath->Add(browseButton, 0, wxALL, 5);

    wxBoxSizer* x_FileIndex = new wxBoxSizer(wxHORIZONTAL);
    x_FileIndex->Add(fileIndexLabel, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    x_FileIndex->Add(m_fileIndex, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

    browseButton->Bind(wxEVT_BUTTON, &PlanetToolWin::OnBrowseFileName, this);
    m_filePathTextCtrl->Bind(wxEVT_TEXT, &PlanetToolWin::OnFileTextChange, this);
    m_fileIndex->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_FileIndex), NULL, this);
#endif

    // All major controls
    wxBoxSizer *topSizer = new wxBoxSizer(wxVERTICAL);
    topSizer->AddSpacer(5);
    topSizer->Add(ParamsSizer, 0, wxEXPAND, 5);
    topSizer->AddSpacer(10);
#if FILE_SIMULATOR_MODE
    topSizer->Add(x_FilePath, 0, wxEXPAND, 5);
    topSizer->Add(x_FileIndex, 0, wxEXPAND, 5);
    topSizer->AddSpacer(10);
#endif
    topSizer->Add(m_EclipseModeCheckBox, 0, wxLEFT | wxALIGN_LEFT, 100);
    topSizer->AddSpacer(10);
    topSizer->Add(m_RoiCheckBox, 0, wxLEFT | wxALIGN_LEFT, 100);
    topSizer->AddSpacer(10);
    topSizer->Add(ThresholdLabel, 0, wxLEFT | wxTOP, 10);
    topSizer->Add(m_ThresholdSlider, 0, wxALL, 10);
    topSizer->AddSpacer(10);
    topSizer->Add(ButtonSizer, 0, wxALL | wxALIGN_CENTER_HORIZONTAL, 5);
    topSizer->AddSpacer(10);
    topSizer->Add(m_ShowElements, 0, wxLEFT | wxALIGN_LEFT, 30);
    topSizer->AddSpacer(10);
    topSizer->Add(m_status, 0, wxALL, 5);

    SetSizer(topSizer);
    Layout();
    topSizer->Fit(this);

    // Connect Events
    m_CloseButton->Bind(wxEVT_COMMAND_BUTTON_CLICKED, &PlanetToolWin::OnCloseButton, this);
    m_CloseButton->Bind(wxEVT_KEY_DOWN, &PlanetToolWin::OnKeyDown, this);
    m_CloseButton->Bind(wxEVT_KEY_UP, &PlanetToolWin::OnKeyUp, this);
    m_CloseButton->Bind(wxEVT_ENTER_WINDOW, &PlanetToolWin::OnMouseEnterCloseBtn, this);
    m_CloseButton->Bind(wxEVT_LEAVE_WINDOW, &PlanetToolWin::OnMouseLeaveCloseBtn, this);
    m_EclipseModeCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnEclipseModeClick, this);
    m_RoiCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnRoiModeClick, this);
    m_ShowElements->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnShowElementsClick, this);
    Bind(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(PlanetToolWin::OnClose), this);
    Bind(APPSTATE_NOTIFY_EVENT, wxCommandEventHandler(PlanetToolWin::OnAppStateNotify), this);

    m_enableButton->Connect(wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler(PlanetToolWin::OnEnableToggled), NULL, this);
    m_minDist->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_minDist), NULL, this);
    m_param1->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_param1), NULL, this);
    m_param2->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_param2), NULL, this);
    m_minRadius->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_minRadius), NULL, this);
    m_maxRadius->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_maxRadius), NULL, this);

    // Set initial values of the planetary tracking state and parameters
    pFrame->pGuider->SetEclipseMode(pConfig->Global.GetInt("/PlanetTool/eclipse_mode", 1));
    pFrame->pGuider->SetPlanetaryParam_minDist(pConfig->Global.GetInt("/PlanetTool/min_dist", PT_MIN_DIST_DEFAULT));
    pFrame->pGuider->SetPlanetaryParam_param1(pConfig->Global.GetInt("/PlanetTool/param1", PT_PARAM1_DEFAULT));
    pFrame->pGuider->SetPlanetaryParam_param2(pConfig->Global.GetInt("/PlanetTool/param2", PT_PARAM2_DEFAULT));
    pFrame->pGuider->SetPlanetaryParam_minRadius(pConfig->Global.GetInt("/PlanetTool/min_radius", PT_MIN_RADIUS_DEFAULT));
    pFrame->pGuider->SetPlanetaryParam_maxRadius(pConfig->Global.GetInt("/PlanetTool/max_radius", PT_MAX_RADIUS_DEFAULT));
    pFrame->pGuider->SetPlanetaryParam_lowThreshold(pConfig->Global.GetInt("/PlanetTool/high_threshold", PT_HIGH_THRESHOLD_DEFAULT)/2);
    pFrame->pGuider->SetPlanetaryParam_highThreshold(pConfig->Global.GetInt("/PlanetTool/high_threshold", PT_HIGH_THRESHOLD_DEFAULT));
    pFrame->pGuider->SetPlanetaryElementsButtonState(false);
    pFrame->pGuider->SetPlanetaryElementsVisual(false);

#if FILE_SIMULATOR_MODE
    SimFileIndex = pConfig->Global.GetInt("/PlanetTool/sim_file_index", 1);
    simulatorFileTemplate = pConfig->Global.GetString("/PlanetTool/sim_filename", _("/Temp/phd2/sim_image.png"));
    m_filePathTextCtrl->SetValue(simulatorFileTemplate);
    m_fileIndex->SetValue(SimFileIndex);
#endif

    m_minDist->SetValue(pFrame->pGuider->GetPlanetaryParam_minDist());
    m_param1->SetValue(pFrame->pGuider->GetPlanetaryParam_param1());
    m_param2->SetValue(pFrame->pGuider->GetPlanetaryParam_param2());
    m_minRadius->SetValue(pFrame->pGuider->GetPlanetaryParam_minRadius());
    m_maxRadius->SetValue(pFrame->pGuider->GetPlanetaryParam_maxRadius());
    m_ThresholdSlider->SetValue(pFrame->pGuider->GetPlanetaryParam_highThreshold());
    m_EclipseModeCheckBox->SetValue(pFrame->pGuider->GetEclipseMode());
    m_RoiCheckBox->SetValue(pFrame->pGuider->GetRoiEnableState());

    SetEnabledState(this, pFrame->pGuider->GetPlanetaryEnableState());

    UpdateStatus();
}

PlanetToolWin::~PlanetToolWin(void)
{
    pFrame->pPlanetTool = 0;
}

void PlanetToolWin::OnEnableToggled(wxCommandEvent& event)
{
    if (!pFrame->pGuider->GetPlanetaryEnableState())
    {
        pFrame->SaveStarFindMode();
        pFrame->SetStarFindMode(Star::FIND_PLANET);
        pFrame->pGuider->SetPlanetaryEnableState(true);
        pFrame->m_PlanetaryMenuItem->Check(true);
        SetEnabledState(this, true);
    } else
    {
        pFrame->RestoreStarFindMode();
        pFrame->pGuider->SetPlanetaryEnableState(false);
        pFrame->m_PlanetaryMenuItem->Check(false);
        SetEnabledState(this, false);
    }

    // Update elements display state
    OnShowElementsClick(event);
}

void PlanetToolWin::OnSpinCtrl_minDist(wxSpinDoubleEvent& event)
{
    int v = m_minDist->GetValue();
    pFrame->pGuider->SetPlanetaryParam_minDist(v < 1 ? 1 : v);
}

void PlanetToolWin::OnSpinCtrl_param1(wxSpinDoubleEvent& event)
{
    int v = m_param1->GetValue();
    pFrame->pGuider->SetPlanetaryParam_param1(v < 1 ? 1 : v);
}

void PlanetToolWin::OnSpinCtrl_param2(wxSpinDoubleEvent& event)
{
    int v = m_param2->GetValue();
    pFrame->pGuider->SetPlanetaryParam_param2(v < 1 ? 1 : v);
}

void PlanetToolWin::OnSpinCtrl_minRadius(wxSpinDoubleEvent& event)
{
    int v = m_minRadius->GetValue();
    pFrame->pGuider->SetPlanetaryParam_minRadius(v < 1 ? 1 : v);
    pFrame->pGuider->PlanetVisualRefresh();
}

void PlanetToolWin::OnSpinCtrl_maxRadius(wxSpinDoubleEvent& event)
{
    int v = m_maxRadius->GetValue();
    pFrame->pGuider->SetPlanetaryParam_maxRadius(v < 1 ? 1 : v);
    pFrame->pGuider->PlanetVisualRefresh();
}

void PlanetToolWin::OnEclipseModeClick(wxCommandEvent& event)
{
    bool EclipseMode = m_EclipseModeCheckBox->IsChecked();
    pFrame->pGuider->SetEclipseMode(EclipseMode);

    bool enabled = pFrame->pGuider->GetPlanetaryEnableState();
    m_ThresholdSlider->Enable(enabled && EclipseMode);

    // These parameters aren't used in eclipse mode
    m_minDist->Enable(!EclipseMode);
    m_param1->Enable(!EclipseMode);
    m_param2->Enable(!EclipseMode);

    // Update elements display state
    OnShowElementsClick(event);
}

void PlanetToolWin::OnRoiModeClick(wxCommandEvent& event)
{
    bool enabled = m_RoiCheckBox->IsChecked();
    pFrame->pGuider->SetRoiEnableState(enabled);
}

void PlanetToolWin::OnShowElementsClick(wxCommandEvent& event)
{
    bool enabled = m_ShowElements->IsChecked();
    pFrame->pGuider->SetPlanetaryElementsButtonState(enabled);
    if (pFrame->pGuider->GetPlanetaryEnableState() && enabled)
        pFrame->pGuider->SetPlanetaryElementsVisual(true);
    else
        pFrame->pGuider->SetPlanetaryElementsVisual(false);
    pFrame->pGuider->Refresh();
    pFrame->pGuider->Update();
}

void PlanetToolWin::UpdateStatus()
{
    bool enabled = pFrame->pGuider->GetPlanetaryEnableState();
    if (enabled)
        m_status->SetValue(_("Center the planet in the guiding camera.\n"
            "Select it, do calibration and then start guiding.\n"));
    else
        m_status->SetValue(_("In star tracking mode\n"));

    wxCommandEvent evt_none;
    OnEclipseModeClick(evt_none);
}

void PlanetToolWin::OnAppStateNotify(wxCommandEvent& event)
{
}

void PlanetToolWin::OnKeyDown(wxKeyEvent& event)
{
    if (event.AltDown() && m_MouseHoverFlag) {
        m_CloseButton->SetLabel(wxT("Reset"));
    }
    event.Skip(); // Ensure that other key handlers are not skipped
}

void PlanetToolWin::OnKeyUp(wxKeyEvent& event)
{
    m_CloseButton->SetLabel(wxT("Close"));
    event.Skip();
}

void PlanetToolWin::OnMouseEnterCloseBtn(wxMouseEvent& event)
{
    m_MouseHoverFlag = true;
    if (wxGetKeyState(WXK_ALT)) {
        m_CloseButton->SetLabel(wxT("Reset"));
    }
    event.Skip();
}

void PlanetToolWin::OnMouseLeaveCloseBtn(wxMouseEvent& event)
{
    m_MouseHoverFlag = false;
    m_CloseButton->SetLabel(wxT("Close"));
    event.Skip();
}

void PlanetToolWin::OnThresholdChanged(wxCommandEvent& event)
{
    int value = event.GetInt();
    pFrame->pGuider->SetPlanetaryParam_lowThreshold(value/2);
    pFrame->pGuider->SetPlanetaryParam_highThreshold(value);
}

void PlanetToolWin::OnClose(wxCloseEvent& evt)
{
    pFrame->m_PlanetaryMenuItem->Check(pFrame->pGuider->GetPlanetaryEnableState());
    pFrame->pGuider->SetEclipseMode(m_EclipseModeCheckBox->IsChecked());
    pFrame->pGuider->SetPlanetaryElementsButtonState(false);
    pFrame->pGuider->SetPlanetaryElementsVisual(false);
    pFrame->pGuider->Refresh();
    pFrame->pGuider->Update();

    // save detection parameters
    pConfig->Global.SetInt("/PlanetTool/eclipse_mode", pFrame->pGuider->GetEclipseMode());
    pConfig->Global.SetInt("/PlanetTool/min_dist", pFrame->pGuider->GetPlanetaryParam_minDist());
    pConfig->Global.SetInt("/PlanetTool/param1", pFrame->pGuider->GetPlanetaryParam_param1());
    pConfig->Global.SetInt("/PlanetTool/param2", pFrame->pGuider->GetPlanetaryParam_param2());
    pConfig->Global.SetInt("/PlanetTool/min_radius", pFrame->pGuider->GetPlanetaryParam_minRadius());
    pConfig->Global.SetInt("/PlanetTool/max_radius", pFrame->pGuider->GetPlanetaryParam_maxRadius());
    pConfig->Global.SetInt("/PlanetTool/high_threshold", pFrame->pGuider->GetPlanetaryParam_highThreshold());

#if FILE_SIMULATOR_MODE
    pConfig->Global.SetInt("/PlanetTool/sim_file_index", SimFileIndex);
    pConfig->Global.SetString("/PlanetTool/sim_filename", simulatorFileTemplate);
#endif

    Destroy();
}

void PlanetToolWin::OnCloseButton(wxCommandEvent& event)
{
    // Reset all to defaults
    if (wxGetKeyState(WXK_ALT))
    {
        pFrame->pGuider->SetEclipseMode(false);
        pFrame->pGuider->SetPlanetaryParam_minDist(PT_MIN_DIST_DEFAULT);
        pFrame->pGuider->SetPlanetaryParam_param1(PT_PARAM1_DEFAULT);
        pFrame->pGuider->SetPlanetaryParam_param2(PT_PARAM2_DEFAULT);
        pFrame->pGuider->SetPlanetaryParam_minRadius(PT_MIN_RADIUS_DEFAULT);
        pFrame->pGuider->SetPlanetaryParam_maxRadius(PT_MAX_RADIUS_DEFAULT);
        pFrame->pGuider->SetPlanetaryParam_lowThreshold(PT_HIGH_THRESHOLD_DEFAULT/2);
        pFrame->pGuider->SetPlanetaryParam_highThreshold(PT_HIGH_THRESHOLD_DEFAULT);

        if (pFrame->GetStarFindMode() == Star::FIND_PLANET)
            pFrame->RestoreStarFindMode();        

        SetEnabledState(this, false);
        m_minDist->SetValue(pFrame->pGuider->GetPlanetaryParam_minDist());
        m_param1->SetValue(pFrame->pGuider->GetPlanetaryParam_param1());
        m_param2->SetValue(pFrame->pGuider->GetPlanetaryParam_param2());
        m_minRadius->SetValue(pFrame->pGuider->GetPlanetaryParam_minRadius());
        m_maxRadius->SetValue(pFrame->pGuider->GetPlanetaryParam_maxRadius());
        m_ThresholdSlider->SetValue(pFrame->pGuider->GetPlanetaryParam_highThreshold());
        m_EclipseModeCheckBox->SetValue(false);
    }
    else
        this->Close();
}

#if FILE_SIMULATOR_MODE
void PlanetToolWin::OnBrowseFileName(wxCommandEvent& event)
{
    wxFileDialog openFileDialog(this, wxT("Open File"), wxEmptyString, wxEmptyString, wxT("All Files (*.*)|*.*"), wxFD_OPEN | wxFD_FILE_MUST_EXIST);

    if (openFileDialog.ShowModal() == wxID_OK)
    {
        simulatorFileTemplate = openFileDialog.GetPath();
        m_filePathTextCtrl->SetValue(simulatorFileTemplate);
    }
}

void PlanetToolWin::OnFileTextChange(wxCommandEvent& event)
{
    simulatorFileTemplate = m_filePathTextCtrl->GetValue();
}

void PlanetToolWin::OnSpinCtrl_FileIndex(wxSpinDoubleEvent& event)
{
    double v = m_fileIndex->GetValue();
    if (v > 9999)
        v = 9999;
    SimFileIndex = v < 0 ? 0 : v;
}
#endif

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
