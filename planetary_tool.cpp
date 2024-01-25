/*
 *  planetary_tool.cpp
 *  PHD Guiding

 *  Created by Leo Shatz.
 *  Copyright (c) 2023-2024 Leo Shatz, openphdguiding.org
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
    GuiderPlanet* pPlanet;

    wxNotebook* m_tabs;
    wxPanel* m_planetTab;
    wxPanel* m_featuresTab;
    wxCheckBox* m_enableCheckBox;
    wxCheckBox* m_featureTrackingCheckBox;

#if USE_PLANETARY_STATUS
    wxStaticText *m_status;
#endif

    wxSpinCtrlDouble *m_minDist;
    wxSpinCtrlDouble *m_param1;
    wxSpinCtrlDouble *m_param2;
    wxSpinCtrlDouble *m_minRadius;
    wxSpinCtrlDouble *m_maxRadius;

    wxSlider *m_ThresholdSlider;
    wxSlider *m_minHessianSlider;
    wxSlider *m_maxFeaturesSlider;

    wxButton   *m_CloseButton;
    wxCheckBox *m_EclipseModeCheckBox;
    wxCheckBox *m_RoiCheckBox;
    wxCheckBox *m_ShowElements;
    bool        m_MouseHoverFlag;

    PlanetToolWin();
    ~PlanetToolWin();

    void OnClose(wxCloseEvent& event);
    void OnCloseButton(wxCommandEvent& event);
    void OnKeyDown(wxKeyEvent& event);
    void OnKeyUp(wxKeyEvent& event);
    void OnMouseEnterCloseBtn(wxMouseEvent& event);
    void OnMouseLeaveCloseBtn(wxMouseEvent& event);
    void OnThresholdChanged(wxCommandEvent& event);
    void OnMinHessianChanged(wxCommandEvent& event);
    void OnMaxFeaturesChanged(wxCommandEvent& event);
    void OnSurfaceTrackingClick(wxCommandEvent& event);

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
    win->SetTitle(wxGetTranslation(active ? TITLE_ACTIVE : TITLE));
    win->UpdateStatus();
}

PlanetToolWin::PlanetToolWin()
    : wxDialog(pFrame, wxID_ANY, wxGetTranslation(TITLE), wxDefaultPosition, wxDefaultSize, wxCAPTION | wxCLOSE_BOX),
      pPlanet(&pFrame->pGuider->m_Planet)
{
    SetSizeHints(wxDefaultSize, wxDefaultSize);

    // Set custom duration of tooltip display to 10 seconds
    wxToolTip::SetAutoPop(10000);

    m_tabs = new wxNotebook(this, wxID_ANY);
    m_planetTab = new wxPanel(m_tabs, wxID_ANY);
    m_tabs->AddPage(m_planetTab, "Planetary tracking", true);

    m_featuresTab = new wxPanel(m_tabs, wxID_ANY);
    m_tabs->AddPage(m_featuresTab, "Surface features tracking", false);
    m_enableCheckBox = new wxCheckBox(this, wxID_ANY, _("Enable planetary tracking"));
    m_enableCheckBox->SetToolTip(_("Toggle star/planetary tracking mode"));

    m_featureTrackingCheckBox = new wxCheckBox(this, wxID_ANY, _("Enable surface features detection/tracking"));
    m_featureTrackingCheckBox->SetToolTip(_("Enable surface feature detection/tracking mode for imaging at high magnification"));

    wxStaticText* minDist_Label = new wxStaticText(m_planetTab, wxID_ANY, _("min dist:"));
    minDist_Label->Wrap(-1);
    wxStaticText* param1_Label = new wxStaticText(m_planetTab, wxID_ANY, _("p1:"));
    param1_Label->Wrap(-1);
    wxStaticText* param2_Label = new wxStaticText(m_planetTab, wxID_ANY, _("p2:"));
    param2_Label->Wrap(-1);
    wxStaticText* minRadius_Label = new wxStaticText(m_planetTab, wxID_ANY, _("min radius:"));
    minRadius_Label->Wrap(-1);
    wxStaticText* maxRadius_Label = new wxStaticText(m_planetTab, wxID_ANY, _("max radius:"));
    maxRadius_Label->Wrap(-1);

    m_minDist = new wxSpinCtrlDouble(m_planetTab, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(80, -1), wxSP_ARROW_KEYS, 1, 1024, PT_MIN_DIST_DEFAULT);
    minDist_Label->SetToolTip(_("minimum distance between the centers of the detected circles"));

    m_param1 = new wxSpinCtrlDouble(m_planetTab, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(80, -1), wxSP_ARROW_KEYS, 1, 255, PT_PARAM1_DEFAULT);
    param1_Label->SetToolTip(_("The higher threshold for the Canny edge detector. Increase this value to avoid false circles"));

    m_param2 = new wxSpinCtrlDouble(m_planetTab, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(80, -1), wxSP_ARROW_KEYS, 1, 512, PT_PARAM2_DEFAULT);
    param2_Label->SetToolTip(_("The accumulator threshold for circle centers. Smaller values will mean more circle candidates, "
        "and larger values will suppress weaker circles. You might want to increase this value if you're getting false circles"));

    m_minRadius = new wxSpinCtrlDouble(m_planetTab, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(80, -1), wxSP_ARROW_KEYS, 1, 1024, PT_MIN_RADIUS_DEFAULT);
    minRadius_Label->SetToolTip(_("Minimum planet radius in pixels. If set to 0, the minimal size is not limited."));

    m_maxRadius = new wxSpinCtrlDouble(m_planetTab, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(80, -1), wxSP_ARROW_KEYS, 1, 1024, PT_MAX_RADIUS_DEFAULT);
    maxRadius_Label->SetToolTip(_("Maximum planet radius in pixels. If set to 0, the maximal size is not limited. "
        "If neither minRadius nor maxRadius is set, they are estimated from the image size."));

    wxBoxSizer *x_circleParams = new wxBoxSizer(wxHORIZONTAL);
    x_circleParams->Add(0, 0, 1, wxEXPAND, 5);
    x_circleParams->Add(minDist_Label, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    x_circleParams->Add(m_minDist, 0, wxALIGN_CENTER_VERTICAL, 5);
    x_circleParams->Add(0, 0, 1, wxEXPAND, 5);
    x_circleParams->Add(param1_Label, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    x_circleParams->Add(m_param1, 0, wxALIGN_CENTER_VERTICAL, 5);
    x_circleParams->Add(0, 0, 1, wxEXPAND, 5);
    x_circleParams->Add(param2_Label, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    x_circleParams->Add(m_param2, 0, wxALIGN_CENTER_VERTICAL, 5);
    x_circleParams->Add(0, 0, 1, wxEXPAND, 5);

    wxBoxSizer *x_radii = new wxBoxSizer(wxHORIZONTAL);
    x_radii->Add(0, 0, 1, wxEXPAND, 5);
    x_radii->Add(minRadius_Label, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    x_radii->Add(m_minRadius, 0, wxALIGN_CENTER_VERTICAL, 5);
    x_radii->Add(0, 0, 1, wxEXPAND, 5);
    x_radii->Add(maxRadius_Label, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    x_radii->Add(m_maxRadius, 0, wxALIGN_CENTER_VERTICAL, 5);
    x_radii->Add(0, 0, 1, wxEXPAND, 5);

    // Eclipse mode stuff
    m_EclipseModeCheckBox = new wxCheckBox(m_planetTab, wxID_ANY, _("Enable Eclipse mode"));
    m_EclipseModeCheckBox->SetToolTip(_("Enable Eclipse mode for enhanced tracking of partial solar / lunar discs"));
    wxStaticText* ThresholdLabel = new wxStaticText(m_planetTab, wxID_ANY, wxT("Edge Detection Threshold:"), wxDefaultPosition, wxDefaultSize, 0);
    m_ThresholdSlider = new wxSlider(m_planetTab, wxID_ANY, PT_HIGH_THRESHOLD_DEFAULT, PT_HIGH_THRESHOLD_MIN, PT_HIGH_THRESHOLD_MAX, wxPoint(20, 20), wxSize(400, -1), wxSL_HORIZONTAL | wxSL_LABELS);
    ThresholdLabel->SetToolTip(_("Higher values reduce sensitivity to weaker edges, providing cleaner edge maps. Detected edges are shown in red."));
    m_ThresholdSlider->Bind(wxEVT_SLIDER, &PlanetToolWin::OnThresholdChanged, this);
    m_RoiCheckBox = new wxCheckBox(m_planetTab, wxID_ANY, _("Enable ROI"));
    m_RoiCheckBox->SetToolTip(_("Enable ROI for improved processing speed and reduced CPU usage."));

    // Show/hide detected elements
    m_ShowElements = new wxCheckBox(this, wxID_ANY, _("Display internal edges/features"));
    m_ShowElements->SetToolTip(_("Toggle the visibility of internally detected edges/features and tune detection parameters "
        "to maintain a manageable number of these features while keeping them as close as possible to the light disk boundary."));

    // Add all planetary tab elements
    wxStaticBoxSizer *planetSizer = new wxStaticBoxSizer(new wxStaticBox(m_planetTab, wxID_ANY, _("")), wxVERTICAL);
    planetSizer->AddSpacer(20);
    planetSizer->Add(m_EclipseModeCheckBox, 0, wxLEFT | wxALIGN_LEFT, 20);
    planetSizer->AddSpacer(10);
    planetSizer->Add(m_RoiCheckBox, 0, wxLEFT | wxALIGN_LEFT, 20);
    planetSizer->AddSpacer(20);
    planetSizer->Add(x_circleParams, 0, wxEXPAND, 5);
    planetSizer->AddSpacer(20);
    planetSizer->Add(x_radii, 0, wxEXPAND, 5);
    planetSizer->AddSpacer(20);
    planetSizer->Add(ThresholdLabel, 0, wxLEFT | wxTOP, 10);
    planetSizer->Add(m_ThresholdSlider, 0, wxALL, 10);
    m_planetTab->SetSizer(planetSizer);
    m_planetTab->Layout();

    // Surface tracking elements
    wxStaticText* minHessianLabel = new wxStaticText(m_featuresTab, wxID_ANY, wxT("Detection Sensitivity:"), wxDefaultPosition, wxDefaultSize, 0);
    m_minHessianSlider = new wxSlider(m_featuresTab, wxID_ANY, PT_MIN_HESSIAN_MAX - PT_MIN_HESSIAN_DEFAULT, 0, PT_MIN_HESSIAN_MAX, wxPoint(20, 20), wxSize(400, -1), wxSL_HORIZONTAL | wxSL_LABELS);
    minHessianLabel->SetToolTip(_("Adjusts the sensitivity of feature detection. A lower value detects fewer but more robust features. "
                                  "Higher values increase the number of detected features but may include more noise. "
                                  "Ideal value depends on image content and quality"));
    wxStaticText* maxFeaturesLabel = new wxStaticText(m_featuresTab, wxID_ANY, wxT("Maximum number of surface features:"), wxDefaultPosition, wxDefaultSize, 0);
    m_maxFeaturesSlider = new wxSlider(m_featuresTab, wxID_ANY, PT_MAX_SURFACE_FEATURES, 10, PT_MAX_SURFACE_FEATURES, wxPoint(20, 20), wxSize(400, -1), wxSL_HORIZONTAL | wxSL_LABELS);
    maxFeaturesLabel->SetToolTip(_("Limits maximum number of features used for tracking."));
    m_minHessianSlider->Bind(wxEVT_SLIDER, &PlanetToolWin::OnMinHessianChanged, this);
    m_maxFeaturesSlider->Bind(wxEVT_SLIDER, &PlanetToolWin::OnMaxFeaturesChanged, this);
    wxStaticBoxSizer* surfaceSizer = new wxStaticBoxSizer(new wxStaticBox(m_featuresTab, wxID_ANY, _("")), wxVERTICAL);
    surfaceSizer->AddSpacer(10);
    surfaceSizer->Add(minHessianLabel, 0, wxLEFT | wxTOP, 10);
    surfaceSizer->Add(m_minHessianSlider, 0, wxALL, 10);
    surfaceSizer->AddSpacer(10);
    surfaceSizer->Add(maxFeaturesLabel, 0, wxLEFT | wxTOP, 10);
    surfaceSizer->Add(m_maxFeaturesSlider, 0, wxALL, 10);
    surfaceSizer->AddSpacer(10);

    m_featuresTab->SetSizer(surfaceSizer);
    m_featuresTab->Layout();

#if USE_PLANETARY_STATUS
    // Status area
    wxPanel* bezelPanel = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxSize(420, 90), wxBORDER_STATIC);
    wxBoxSizer* panelSizer = new wxBoxSizer(wxVERTICAL);
    m_status = new wxStaticText(bezelPanel, wxID_ANY, _(""));
    panelSizer->Add(m_status, 0, wxLEFT | wxALIGN_LEFT, 10);
    bezelPanel->SetSizer(panelSizer);
#endif

    // Close button
    m_MouseHoverFlag = false;
    wxBoxSizer *ButtonSizer = new wxBoxSizer(wxHORIZONTAL);
    m_CloseButton = new wxButton(this, wxID_ANY, _("Close"));
    ButtonSizer->Add(m_CloseButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

#if FILE_SIMULATOR_MODE
    bool IsSimCam = pCamera && pCamera->Name == "Simulator";
    wxBoxSizer* x_FilePath;
    wxBoxSizer* x_FileIndex;
    if (IsSimCam)
    {
        wxStaticText* fileFormatLabel = new wxStaticText(this, wxID_ANY, wxT("file"));
        fileFormatLabel->Wrap(-1);
        wxStaticText* fileIndexLabel = new wxStaticText(this, wxID_ANY, wxT("index"));
        fileIndexLabel->Wrap(-1);

        m_filePathTextCtrl = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(250, -1));
        wxButton* browseButton = new wxButton(this, wxID_ANY, wxT("Browse"));

        m_fileIndex = new wxSpinCtrlDouble(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 9999, 1);

        x_FilePath = new wxBoxSizer(wxHORIZONTAL);
        x_FilePath->Add(0, 0, 1, wxEXPAND, 5);
        x_FilePath->Add(fileFormatLabel, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
        x_FilePath->Add(m_filePathTextCtrl, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
        x_FilePath->Add(browseButton, 0, wxALL, 5);

        x_FileIndex = new wxBoxSizer(wxHORIZONTAL);
        x_FileIndex->Add(fileIndexLabel, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
        x_FileIndex->Add(m_fileIndex, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

        browseButton->Bind(wxEVT_BUTTON, &PlanetToolWin::OnBrowseFileName, this);
        m_filePathTextCtrl->Bind(wxEVT_TEXT, &PlanetToolWin::OnFileTextChange, this);
        m_fileIndex->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_FileIndex), NULL, this);
    }
#endif

    // All top level controls
    wxBoxSizer *topSizer = new wxBoxSizer(wxVERTICAL);
    topSizer->AddSpacer(10);
    topSizer->Add(m_enableCheckBox, 0, wxLEFT | wxALIGN_LEFT, 20);
    topSizer->AddSpacer(10);
    topSizer->Add(m_featureTrackingCheckBox, 0, wxLEFT | wxALIGN_LEFT, 20);
    topSizer->AddSpacer(10);
    topSizer->Add(m_tabs, 1, wxEXPAND | wxALL, 10);
    topSizer->AddSpacer(10);
#if FILE_SIMULATOR_MODE
    if (IsSimCam)
    {
        topSizer->Add(x_FilePath, 0, wxLEFT | wxALIGN_LEFT, 10);
        topSizer->Add(x_FileIndex, 0, wxLEFT | wxALIGN_LEFT, 10);
        topSizer->AddSpacer(10);
    }
#endif
    topSizer->Add(m_ShowElements, 0, wxLEFT | wxALIGN_LEFT, 20);
    topSizer->AddSpacer(10);
#if USE_PLANETARY_STATUS
    topSizer->Add(bezelPanel, 0, wxALL | wxALIGN_CENTER_HORIZONTAL, 10);
#endif
    topSizer->Add(ButtonSizer, 0, wxALL | wxALIGN_CENTER_HORIZONTAL, 10);

    SetSizer(topSizer);
    Layout();
    topSizer->Fit(this);

    // Connect Events
    m_enableCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnEnableToggled, this);
    m_featureTrackingCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnSurfaceTrackingClick, this);
    m_CloseButton->Bind(wxEVT_COMMAND_BUTTON_CLICKED, &PlanetToolWin::OnCloseButton, this);
    m_CloseButton->Bind(wxEVT_KEY_DOWN, &PlanetToolWin::OnKeyDown, this);
    m_CloseButton->Bind(wxEVT_KEY_UP, &PlanetToolWin::OnKeyUp, this);
    m_CloseButton->Bind(wxEVT_ENTER_WINDOW, &PlanetToolWin::OnMouseEnterCloseBtn, this);
    m_CloseButton->Bind(wxEVT_LEAVE_WINDOW, &PlanetToolWin::OnMouseLeaveCloseBtn, this);
    m_EclipseModeCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnEclipseModeClick, this);
    m_RoiCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnRoiModeClick, this);
    m_ShowElements->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnShowElementsClick, this);
    Bind(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(PlanetToolWin::OnClose), this);

    m_minDist->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_minDist), NULL, this);
    m_param1->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_param1), NULL, this);
    m_param2->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_param2), NULL, this);
    m_minRadius->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_minRadius), NULL, this);
    m_maxRadius->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_maxRadius), NULL, this);

    // Set initial values of the planetary tracking state and parameters
    pPlanet->SetSurfaceTrackingState(pConfig->Global.GetInt("/PlanetTool/surface_tracking", 0));
    pPlanet->SetEclipseMode(pConfig->Global.GetInt("/PlanetTool/eclipse_mode", 1));
    pPlanet->SetPlanetaryParam_minDist(pConfig->Global.GetInt("/PlanetTool/min_dist", PT_MIN_DIST_DEFAULT));
    pPlanet->SetPlanetaryParam_param1(pConfig->Global.GetInt("/PlanetTool/param1", PT_PARAM1_DEFAULT));
    pPlanet->SetPlanetaryParam_param2(pConfig->Global.GetInt("/PlanetTool/param2", PT_PARAM2_DEFAULT));
    pPlanet->SetPlanetaryParam_minRadius(pConfig->Global.GetInt("/PlanetTool/min_radius", PT_MIN_RADIUS_DEFAULT));
    pPlanet->SetPlanetaryParam_maxRadius(pConfig->Global.GetInt("/PlanetTool/max_radius", PT_MAX_RADIUS_DEFAULT));
    pPlanet->SetPlanetaryParam_lowThreshold(pConfig->Global.GetInt("/PlanetTool/high_threshold", PT_HIGH_THRESHOLD_DEFAULT/2));
    pPlanet->SetPlanetaryParam_highThreshold(pConfig->Global.GetInt("/PlanetTool/high_threshold", PT_HIGH_THRESHOLD_DEFAULT));
    pPlanet->SetPlanetaryParam_minHessian(pConfig->Global.GetInt("/PlanetTool/min_hessian", PT_MIN_HESSIAN_DEFAULT));
    pPlanet->SetPlanetaryParam_maxFeatures(pConfig->Global.GetInt("/PlanetTool/max_features", PT_MAX_SURFACE_FEATURES));

    pPlanet->SetPlanetaryElementsButtonState(false);
    pPlanet->SetPlanetaryElementsVisual(false);

#if FILE_SIMULATOR_MODE
    if (IsSimCam)
    {
        SimFileIndex = pConfig->Global.GetInt("/PlanetTool/sim_file_index", 1);
        simulatorFileTemplate = pConfig->Global.GetString("/PlanetTool/sim_filename", _("/Temp/phd2/sim_image.png"));
        m_filePathTextCtrl->SetValue(simulatorFileTemplate);
        m_fileIndex->SetValue(SimFileIndex);
    }
#endif

    m_minDist->SetValue(pPlanet->GetPlanetaryParam_minDist());
    m_param1->SetValue(pPlanet->GetPlanetaryParam_param1());
    m_param2->SetValue(pPlanet->GetPlanetaryParam_param2());
    m_minRadius->SetValue(pPlanet->GetPlanetaryParam_minRadius());
    m_maxRadius->SetValue(pPlanet->GetPlanetaryParam_maxRadius());
    m_ThresholdSlider->SetValue(pPlanet->GetPlanetaryParam_highThreshold());
    m_minHessianSlider->SetValue(pPlanet->GetPlanetaryParam_minHessian());
    m_maxFeaturesSlider->SetValue(pPlanet->GetPlanetaryParam_maxFeatures());
    m_featureTrackingCheckBox->SetValue(pPlanet->GetSurfaceTrackingState());
    m_EclipseModeCheckBox->SetValue(pPlanet->GetEclipseMode());
    m_RoiCheckBox->SetValue(pPlanet->GetRoiEnableState());
    m_enableCheckBox->SetValue(pPlanet->GetPlanetaryEnableState());

    SetEnabledState(this, pPlanet->GetPlanetaryEnableState());

    m_tabs->SetSelection(pPlanet->GetSurfaceTrackingState() ? 1 : 0);

    int xpos = pConfig->Global.GetInt("/PlanetTool/pos.x", -1);
    int ypos = pConfig->Global.GetInt("/PlanetTool/pos.y", -1);
    if (wxGetKeyState(WXK_ALT))
    {
        xpos = -1;
        ypos = -1;
    }

    MyFrame::PlaceWindowOnScreen(this, xpos, ypos);

    UpdateStatus();
}

PlanetToolWin::~PlanetToolWin(void)
{
    pFrame->pPlanetTool = 0;
}

void PlanetToolWin::OnEnableToggled(wxCommandEvent& event)
{
    if (m_enableCheckBox->IsChecked())
    {
        pFrame->SaveStarFindMode();
        pFrame->SetStarFindMode(Star::FIND_PLANET);
        pPlanet->SetPlanetaryEnableState(true);
        pFrame->m_PlanetaryMenuItem->Check(true);
        SetEnabledState(this, true);
    } else
    {
        pFrame->RestoreStarFindMode();
        pPlanet->SetPlanetaryEnableState(false);
        pFrame->m_PlanetaryMenuItem->Check(false);
        SetEnabledState(this, false);
    }

    // Update elements display state
    m_tabs->SetSelection(pPlanet->GetSurfaceTrackingState() ? 1 : 0);
    OnShowElementsClick(event);
}

// Toggle surface features detection/tracking mode
void PlanetToolWin::OnSurfaceTrackingClick(wxCommandEvent& event)
{
    bool featureTracking = m_featureTrackingCheckBox->IsChecked();
    pPlanet->SetSurfaceTrackingState(featureTracking);
    m_tabs->SetSelection(featureTracking ? 1 : 0);
    UpdateStatus();
}

void PlanetToolWin::OnSpinCtrl_minDist(wxSpinDoubleEvent& event)
{
    int v = m_minDist->GetValue();
    pPlanet->SetPlanetaryParam_minDist(v < 1 ? 1 : v);
}

void PlanetToolWin::OnSpinCtrl_param1(wxSpinDoubleEvent& event)
{
    int v = m_param1->GetValue();
    pPlanet->SetPlanetaryParam_param1(v < 1 ? 1 : v);
}

void PlanetToolWin::OnSpinCtrl_param2(wxSpinDoubleEvent& event)
{
    int v = m_param2->GetValue();
    pPlanet->SetPlanetaryParam_param2(v < 1 ? 1 : v);
}

void PlanetToolWin::OnSpinCtrl_minRadius(wxSpinDoubleEvent& event)
{
    int v = m_minRadius->GetValue();
    pPlanet->SetPlanetaryParam_minRadius(v < 1 ? 1 : v);
    pPlanet->PlanetVisualRefresh();
}

void PlanetToolWin::OnSpinCtrl_maxRadius(wxSpinDoubleEvent& event)
{
    int v = m_maxRadius->GetValue();
    pPlanet->SetPlanetaryParam_maxRadius(v < 1 ? 1 : v);
    pPlanet->PlanetVisualRefresh();
}

void PlanetToolWin::OnEclipseModeClick(wxCommandEvent& event)
{
    bool EclipseMode = m_EclipseModeCheckBox->IsChecked();
    pPlanet->SetEclipseMode(EclipseMode);
    UpdateStatus();
}

void PlanetToolWin::OnRoiModeClick(wxCommandEvent& event)
{
    bool enabled = m_RoiCheckBox->IsChecked();
    pPlanet->SetRoiEnableState(enabled);
}

void PlanetToolWin::OnShowElementsClick(wxCommandEvent& event)
{
    bool enabled = m_ShowElements->IsChecked();
    pPlanet->SetPlanetaryElementsButtonState(enabled);
    if (pPlanet->GetPlanetaryEnableState() && enabled)
        pPlanet->SetPlanetaryElementsVisual(true);
    else
        pPlanet->SetPlanetaryElementsVisual(false);
    pFrame->pGuider->Refresh();
    pFrame->pGuider->Update();
}

void PlanetToolWin::UpdateStatus()
{
    bool enabled = pPlanet->GetPlanetaryEnableState();
    bool surfaceTracking = pPlanet->GetSurfaceTrackingState();
#if USE_PLANETARY_STATUS
    if (enabled)
        m_status->SetLabel(_("In planetary tracking mode.\n"
            "Center the planet in the guiding camera,\n"
            "select it, run calibration and start guiding.\n"));
    else
        m_status->SetLabel(_("In star tracking mode.\n"));
#endif

    // Update planetary tracking controls
    bool EclipseMode = pPlanet->GetEclipseMode();
    m_featureTrackingCheckBox->Enable(enabled);
    m_minDist->Enable(enabled && !surfaceTracking && !EclipseMode);
    m_param1->Enable(enabled && !surfaceTracking && !EclipseMode);
    m_param2->Enable(enabled && !surfaceTracking && !EclipseMode);
    m_minRadius->Enable(enabled && !surfaceTracking);
    m_maxRadius->Enable(enabled && !surfaceTracking);
    m_EclipseModeCheckBox->Enable(enabled && !surfaceTracking);
    m_RoiCheckBox->Enable(enabled && !surfaceTracking);
    m_ShowElements->Enable(enabled);

    // Update slider states
    m_ThresholdSlider->Enable(enabled && !surfaceTracking && EclipseMode);
    m_minHessianSlider->Enable(enabled && surfaceTracking);
    m_maxFeaturesSlider->Enable(enabled && surfaceTracking);

    // Update tabs state
    m_featuresTab->Enable(surfaceTracking);
    m_planetTab->Enable(!surfaceTracking);
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
    if (value < PT_HIGH_THRESHOLD_MIN)
        value = PT_HIGH_THRESHOLD_MIN;
    if (value > PT_HIGH_THRESHOLD_MAX)
        value = PT_HIGH_THRESHOLD_MAX;
    pPlanet->SetPlanetaryParam_lowThreshold(value/2);
    pPlanet->SetPlanetaryParam_highThreshold(value);
}

void PlanetToolWin::OnMinHessianChanged(wxCommandEvent& event)
{
    int value = event.GetInt();
    pPlanet->SetPlanetaryParam_minHessian(value);
}

void PlanetToolWin::OnMaxFeaturesChanged(wxCommandEvent& event)
{
    int value = event.GetInt();
    pPlanet->SetPlanetaryParam_maxFeatures(value);
}

void PlanetToolWin::OnClose(wxCloseEvent& evt)
{
    pFrame->m_PlanetaryMenuItem->Check(pPlanet->GetPlanetaryEnableState());
    pPlanet->SetEclipseMode(m_EclipseModeCheckBox->IsChecked());
    pPlanet->SetPlanetaryElementsButtonState(false);
    pPlanet->SetPlanetaryElementsVisual(false);
    pFrame->pGuider->Refresh();
    pFrame->pGuider->Update();

    // save detection parameters
    pConfig->Global.SetInt("/PlanetTool/surface_tracking", pPlanet->GetSurfaceTrackingState());
    pConfig->Global.SetInt("/PlanetTool/eclipse_mode", pPlanet->GetEclipseMode());
    pConfig->Global.SetInt("/PlanetTool/min_dist", pPlanet->GetPlanetaryParam_minDist());
    pConfig->Global.SetInt("/PlanetTool/param1", pPlanet->GetPlanetaryParam_param1());
    pConfig->Global.SetInt("/PlanetTool/param2", pPlanet->GetPlanetaryParam_param2());
    pConfig->Global.SetInt("/PlanetTool/min_radius", pPlanet->GetPlanetaryParam_minRadius());
    pConfig->Global.SetInt("/PlanetTool/max_radius", pPlanet->GetPlanetaryParam_maxRadius());
    pConfig->Global.SetInt("/PlanetTool/high_threshold", pPlanet->GetPlanetaryParam_highThreshold());
    pConfig->Global.SetInt("/PlanetTool/min_hessian", pPlanet->GetPlanetaryParam_minHessian());
    pConfig->Global.SetInt("/PlanetTool/max_features", pPlanet->GetPlanetaryParam_maxFeatures());

#if FILE_SIMULATOR_MODE
    pConfig->Global.SetInt("/PlanetTool/sim_file_index", SimFileIndex);
    pConfig->Global.SetString("/PlanetTool/sim_filename", simulatorFileTemplate);
#endif

    // save the window position
    int x, y;
    GetPosition(&x, &y);
    pConfig->Global.SetInt("/PlanetTool/pos.x", x);
    pConfig->Global.SetInt("/PlanetTool/pos.y", y);

    // Revert to a default duration of tooltip display (apparently 5 seconds)
    wxToolTip::SetAutoPop(5000);

    Destroy();
}

void PlanetToolWin::OnCloseButton(wxCommandEvent& event)
{
    // Reset all to defaults
    if (wxGetKeyState(WXK_ALT))
    {
        pPlanet->SetPlanetaryParam_minDist(PT_MIN_DIST_DEFAULT);
        pPlanet->SetPlanetaryParam_param1(PT_PARAM1_DEFAULT);
        pPlanet->SetPlanetaryParam_param2(PT_PARAM2_DEFAULT);
        pPlanet->SetPlanetaryParam_minRadius(PT_MIN_RADIUS_DEFAULT);
        pPlanet->SetPlanetaryParam_maxRadius(PT_MAX_RADIUS_DEFAULT);
        pPlanet->SetPlanetaryParam_lowThreshold(PT_HIGH_THRESHOLD_DEFAULT/2);
        pPlanet->SetPlanetaryParam_highThreshold(PT_HIGH_THRESHOLD_DEFAULT);
        pPlanet->SetPlanetaryParam_minHessian(PT_MIN_HESSIAN_DEFAULT);
        pPlanet->SetPlanetaryParam_maxFeatures(PT_MAX_SURFACE_FEATURES);

        m_minDist->SetValue(pPlanet->GetPlanetaryParam_minDist());
        m_param1->SetValue(pPlanet->GetPlanetaryParam_param1());
        m_param2->SetValue(pPlanet->GetPlanetaryParam_param2());
        m_minRadius->SetValue(pPlanet->GetPlanetaryParam_minRadius());
        m_maxRadius->SetValue(pPlanet->GetPlanetaryParam_maxRadius());
        m_ThresholdSlider->SetValue(pPlanet->GetPlanetaryParam_highThreshold());
        m_minHessianSlider->SetValue(pPlanet->GetPlanetaryParam_minHessian());
        m_maxFeaturesSlider->SetValue(pPlanet->GetPlanetaryParam_maxFeatures());
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
