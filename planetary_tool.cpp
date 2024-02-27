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

struct PlanetToolWin : public wxDialog
{
    GuiderPlanet* pPlanet;

    wxTimer m_advancedTimer;

    wxNotebook* m_tabs;
    wxPanel* m_planetTab;
    wxPanel* m_featuresTab;
    wxBoxSizer* m_advancedSizer;
    wxBoxSizer* m_mainSizer;
    wxCheckBox* m_enableCheckBox;
    wxCheckBox* m_featureTrackingCheckBox;
    wxCheckBox* m_saveVideoLogCheckBox;

    wxRadioButton* m_manualCalibration;
    wxRadioButton* m_autoCalibration;
    wxCheckBox* m_autoBlindGuidingCheckBox;
    wxStaticText* m_BlindGuidingStatus;
    wxButton*   m_calibrateButton;

    wxSpinCtrlDouble *m_minRadius;
    wxSpinCtrlDouble *m_maxRadius;

    wxSlider *m_ThresholdSlider;
    wxSlider *m_minHessianSlider;
    wxSlider *m_maxFeaturesSlider;

    // Controls for camera settings, duplicating the ones from camera setup dialog and exposure time dropdown.
    // Used for streamlining the planetary tracking user experience.
    wxSpinCtrlDouble* m_ExposureCtrl;
    wxSpinCtrlDouble* m_DelayCtrl;
    wxSpinCtrlDouble* m_GainCtrl;
    wxSpinCtrlDouble* m_driftRaGainCtrl;
    wxSpinCtrlDouble* m_driftDecGainCtrl;

    wxButton   *m_CloseButton;
    wxButton   *m_AdvancedButton;
    wxCheckBox *m_RoiCheckBox;
    wxCheckBox *m_ShowElements;
    wxCheckBox* m_NoiseFilter;
    bool        m_MouseHoverFlag;

    PlanetToolWin();
    ~PlanetToolWin();

    void OnAppStateNotify(wxCommandEvent& event);
    void OnAdvancedTimer(wxTimerEvent& event);
    void OnClose(wxCloseEvent& event);
    void OnCloseButton(wxCommandEvent& event);
    void OnAdvancedButton(wxCommandEvent& event);
    void OnKeyDown(wxKeyEvent& event);
    void OnKeyUp(wxKeyEvent& event);
    void OnMouseEnterCloseBtn(wxMouseEvent& event);
    void OnMouseLeaveCloseBtn(wxMouseEvent& event);
    void OnThresholdChanged(wxCommandEvent& event);
    void OnMinHessianChanged(wxCommandEvent& event);
    void OnMaxFeaturesChanged(wxCommandEvent& event);
    void OnSurfaceTrackingClick(wxCommandEvent& event);

    void OnEnableToggled(wxCommandEvent& event);
    void OnSpinCtrl_minRadius(wxSpinDoubleEvent& event);
    void OnSpinCtrl_maxRadius(wxSpinDoubleEvent& event);
    void OnRoiModeClick(wxCommandEvent& event);
    void OnShowElementsClick(wxCommandEvent& event);
    void OnNoiseFilterClick(wxCommandEvent& event);
    void OnBlindGuidingRadioButton(wxCommandEvent& event);
    void OnAutoBlindGuidingCheckBox(wxCommandEvent& event);
    void OnDriftRaGainChanged(wxSpinDoubleEvent& event);
    void OnDriftDecGainChanged(wxSpinDoubleEvent& event);
    void OnBlindGuidingButton(wxCommandEvent& event);

    void OnExposureChanged(wxSpinDoubleEvent& event);
    void OnDelayChanged(wxSpinDoubleEvent& event);
    void OnGainChanged(wxSpinDoubleEvent& event);
    void OnSaveVideoLog(wxCommandEvent& event);

    void UpdateBlindGuidingControls();
    void UpdateStatus();
};

static wxString TITLE = wxTRANSLATE("Planetary tracking | disabled");
static wxString TITLE_ACTIVE = wxTRANSLATE("Planetary tracking | enabled");

static void SetEnabledState(PlanetToolWin* win, bool active)
{
    win->SetTitle(wxGetTranslation(active ? TITLE_ACTIVE : TITLE));
    win->UpdateStatus();
}

// Utility function to add the <label, input> pairs to a flexgrid
static void AddTableEntryPair(wxWindow* parent, wxFlexGridSizer* pTable, const wxString& label, wxWindow* pControl, const wxString& tooltip)
{
    wxStaticText* pLabel = new wxStaticText(parent, wxID_ANY, label + _(": "), wxPoint(-1, -1), wxSize(-1, -1));
    pLabel->SetToolTip(tooltip);
    pTable->Add(pLabel, 1, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    pTable->Add(pControl, 1, wxALL | wxALIGN_CENTER_VERTICAL, 5);
}

static wxSpinCtrlDouble* NewSpinner(wxWindow* parent, wxString formatstr, double val, double minval, double maxval, double inc)
{
    wxSize sz = pFrame->GetTextExtent(wxString::Format(formatstr, maxval));
    wxSpinCtrlDouble* pNewCtrl = pFrame->MakeSpinCtrlDouble(parent, wxID_ANY, wxEmptyString, wxDefaultPosition,
        sz, wxSP_ARROW_KEYS, minval, maxval, val, inc);
    pNewCtrl->SetDigits(0);
    return pNewCtrl;
}

PlanetToolWin::PlanetToolWin()
    : wxDialog(pFrame, wxID_ANY, wxGetTranslation(TITLE), wxDefaultPosition, wxDefaultSize, wxCAPTION | wxCLOSE_BOX),
    m_advancedTimer(this), pPlanet(&pFrame->pGuider->m_Planet), m_MouseHoverFlag(false)

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

    wxStaticText* minRadius_Label = new wxStaticText(m_planetTab, wxID_ANY, _("min radius:"));
    m_minRadius = new wxSpinCtrlDouble(m_planetTab, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(80, -1), wxSP_ARROW_KEYS, PT_RADIUS_MIN, PT_RADIUS_MAX, PT_MIN_RADIUS_DEFAULT);
    minRadius_Label->SetToolTip(_("Minimum planet radius in pixels. If set to 0, the minimal size is not limited."));

    wxStaticText* maxRadius_Label = new wxStaticText(m_planetTab, wxID_ANY, _("max radius:"));
    m_maxRadius = new wxSpinCtrlDouble(m_planetTab, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(80, -1), wxSP_ARROW_KEYS, PT_RADIUS_MIN, PT_RADIUS_MAX, PT_MAX_RADIUS_DEFAULT);
    maxRadius_Label->SetToolTip(_("Maximum planet radius in pixels. If set to 0, the maximal size is not limited. "
        "If neither minRadius nor maxRadius is set, they are estimated from the image size."));

    wxBoxSizer *x_radii = new wxBoxSizer(wxHORIZONTAL);
    x_radii->Add(0, 0, 1, wxEXPAND, 5);
    x_radii->Add(minRadius_Label, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    x_radii->Add(m_minRadius, 0, wxALIGN_CENTER_VERTICAL, 5);
    x_radii->Add(0, 0, 1, wxEXPAND, 5);
    x_radii->Add(maxRadius_Label, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    x_radii->Add(m_maxRadius, 0, wxALIGN_CENTER_VERTICAL, 5);
    x_radii->Add(0, 0, 1, wxEXPAND, 5);

    // Eclipse mode stuff
    wxStaticText* ThresholdLabel = new wxStaticText(m_planetTab, wxID_ANY, wxT("Edge Detection Threshold:"), wxDefaultPosition, wxDefaultSize, 0);
    m_ThresholdSlider = new wxSlider(m_planetTab, wxID_ANY, PT_HIGH_THRESHOLD_DEFAULT, PT_THRESHOLD_MIN, PT_HIGH_THRESHOLD_MAX, wxPoint(20, 20), wxSize(400, -1), wxSL_HORIZONTAL | wxSL_LABELS);
    ThresholdLabel->SetToolTip(_("Higher values reduce sensitivity to weaker edges, providing cleaner edge maps. Detected edges are shown in red."));
    m_ThresholdSlider->Bind(wxEVT_SLIDER, &PlanetToolWin::OnThresholdChanged, this);
    m_RoiCheckBox = new wxCheckBox(m_planetTab, wxID_ANY, _("Enable ROI"));
    m_RoiCheckBox->SetToolTip(_("Enable automatically selected Region Of Interest (ROI) for improved processing speed and reduced CPU usage."));

    // Add all planetary tab elements
    wxStaticBoxSizer *planetSizer = new wxStaticBoxSizer(new wxStaticBox(m_planetTab, wxID_ANY, _("")), wxVERTICAL);
    planetSizer->AddSpacer(20);
    planetSizer->Add(m_RoiCheckBox, 0, wxLEFT | wxALIGN_LEFT, 20);
    planetSizer->AddSpacer(20);
    planetSizer->Add(x_radii, 0, wxEXPAND, 5);
    planetSizer->AddSpacer(10);
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
    m_maxFeaturesSlider = new wxSlider(m_featuresTab, wxID_ANY, PT_MAX_SURFACE_FEATURES, 5, PT_MAX_SURFACE_FEATURES, wxPoint(20, 20), wxSize(400, -1), wxSL_HORIZONTAL | wxSL_LABELS);
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

    // Camera settings group
    wxStaticBoxSizer* pCamGroup = new wxStaticBoxSizer(wxVERTICAL, this, _("Camera settings"));
    wxFlexGridSizer* pCamTable = new wxFlexGridSizer(2, 4, 10, 10);
    m_ExposureCtrl = NewSpinner(this, _T("%4.0f"), 1000, 1, 9999, 1);
    m_GainCtrl = NewSpinner(this, _T("%3.0f"), 0, 0, 100, 1);
    m_DelayCtrl = NewSpinner(this, _T("%5.0f"), 100, 0, 60000, 100);
    m_ExposureCtrl->Bind(wxEVT_SPINCTRLDOUBLE, &PlanetToolWin::OnExposureChanged, this);
    m_GainCtrl->Bind(wxEVT_SPINCTRLDOUBLE, &PlanetToolWin::OnGainChanged, this);
    m_DelayCtrl->Bind(wxEVT_SPINCTRLDOUBLE, &PlanetToolWin::OnDelayChanged, this);
    AddTableEntryPair(this, pCamTable, _("Exposure (ms)"), m_ExposureCtrl, _("Camera exposure in milliseconds)"));
    AddTableEntryPair(this, pCamTable, _("Gain"), m_GainCtrl, _("Camera gain (0-100)"));
    AddTableEntryPair(this, pCamTable, _("Time Lapse (ms)"), m_DelayCtrl,
        _("How long should PHD wait between guide frames? Useful when using very short exposures but wanting to send guide commands less frequently"));
    pCamTable->AddSpacer(10);
    pCamGroup->Add(pCamTable);

    // Show/hide detected elements
    m_ShowElements = new wxCheckBox(this, wxID_ANY, _("Display internal edges/features"));
    m_ShowElements->SetToolTip(_("Toggle the visibility of internally detected edges/features and tune detection parameters "
        "to maintain a manageable number of these features while keeping them as close as possible to the light disk boundary."));
    m_ShowElements->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnShowElementsClick, this);

    // Experimental noise filter
    m_NoiseFilter = new wxCheckBox(this, wxID_ANY, _("Noise suppression filter"));
    m_NoiseFilter->SetToolTip(_("Enable noise filtering only for extremely noisy images. Use this option cautiously, as it's recommended only when absolutely necessary."));
    m_NoiseFilter->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnNoiseFilterClick, this);

    // Video log control
    m_saveVideoLogCheckBox = new wxCheckBox(this, wxID_ANY, _("Enable video log"));
    m_saveVideoLogCheckBox->SetToolTip(_("Enable recording camera frames to video log file during guiding (using SER format)"));
    m_saveVideoLogCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnSaveVideoLog, this);

    // Blind guiding controls
    wxStaticBoxSizer* pBlindGuidingGroup = new wxStaticBoxSizer(wxVERTICAL, this, _("Blind guiding"));
    wxFlexGridSizer* pBlindGuidingTable = new wxFlexGridSizer(3, 2, 5, 5);
    wxString blindToolTip = _("Estimate object position based on RA/DEC drift rates and mount guiding movements. "
        "Manually adjust drift gains when object is still visible in the camera view to keep it centered in a green circle.");
    // Create two mutually exclusive radio buttons : 'Blind Guiding (Manual Calibration)' and 'Automatic Calibration'
    m_manualCalibration = new wxRadioButton(this, wxID_ANY, _("Blind guiding (manual calibration)"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP);
    m_manualCalibration->SetToolTip(blindToolTip);
    m_manualCalibration->Bind(wxEVT_RADIOBUTTON, &PlanetToolWin::OnBlindGuidingRadioButton, this);
    m_autoCalibration = new wxRadioButton(this, wxID_ANY, _("Automatic drift gain calibration"));
    m_autoCalibration->SetToolTip(_("Enable automatic RA / DEC drift gain calibration. "
        "Can be performed only after adjusting detection parameters and carried out during clear weather windows. "
        "The method works by comparing the estimated and actual positions of the object in the camera view."));
    m_autoCalibration->Bind(wxEVT_RADIOBUTTON, &PlanetToolWin::OnBlindGuidingRadioButton, this);
    wxBoxSizer* pBlindCalibrationRadioGroup = new wxBoxSizer(wxVERTICAL);
    pBlindCalibrationRadioGroup->Add(m_manualCalibration, 0, wxLEFT | wxALIGN_LEFT, 5);
    pBlindCalibrationRadioGroup->AddSpacer(10);
    pBlindCalibrationRadioGroup->Add(m_autoCalibration, 0, wxLEFT | wxALIGN_LEFT, 5);
    pBlindCalibrationRadioGroup->AddSpacer(10);
    m_autoBlindGuidingCheckBox = new wxCheckBox(this, wxID_ANY, _("Auto-activate blind guiding on signal loss"));
    m_autoBlindGuidingCheckBox->SetToolTip(_("Enable automatic switch to blind guiding mode on signal loss"));
    m_autoBlindGuidingCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnAutoBlindGuidingCheckBox, this);

    m_driftRaGainCtrl = NewSpinner(this, _T("%.03f"), PT_BLIND_DRIFT_GAIN_DEFAULT, PT_BLIND_DRIFT_GAIN_MIN, PT_BLIND_DRIFT_GAIN_MAX, 0.001);
    m_driftRaGainCtrl->SetDigits(3);
    m_driftDecGainCtrl = NewSpinner(this, _T("%.03f"), PT_BLIND_DRIFT_GAIN_DEFAULT, PT_BLIND_DRIFT_GAIN_MIN, PT_BLIND_DRIFT_GAIN_MAX, 0.001);
    m_driftDecGainCtrl->SetDigits(3);
    m_driftRaGainCtrl->Bind(wxEVT_SPINCTRLDOUBLE, &PlanetToolWin::OnDriftRaGainChanged, this);
    m_driftDecGainCtrl->Bind(wxEVT_SPINCTRLDOUBLE, &PlanetToolWin::OnDriftDecGainChanged, this);
    AddTableEntryPair(this, pBlindGuidingTable, _("RA drift gain"), m_driftRaGainCtrl,
        _("Adjusting this value correctly can help reduce the effective drift rate. Make adjustments gradually (default = 1.00)"));
    AddTableEntryPair(this, pBlindGuidingTable, _("DEC drift gain"), m_driftDecGainCtrl,
        _("Adjusting this value correctly can help reduce the effective drift rate. Make adjustments gradually (default = 1.00)"));

    wxStaticBox* statusBox = new wxStaticBox(this, wxID_ANY, wxT("Blind guiding status"), wxDefaultPosition, wxDefaultSize);
    wxStaticBoxSizer* statusboxSizer = new wxStaticBoxSizer(statusBox, wxVERTICAL);
    m_BlindGuidingStatus = new wxStaticText(this, wxID_ANY, pPlanet->GetBlindCalibrationStatus());
    statusboxSizer->Add(m_BlindGuidingStatus, 1, wxALL, 5);
    statusboxSizer->AddSpacer(10);
    m_calibrateButton = new wxButton(this, wxID_ANY, _("Start Blind Guiding"));
    m_calibrateButton->Bind(wxEVT_COMMAND_BUTTON_CLICKED, &PlanetToolWin::OnBlindGuidingButton, this);
    m_calibrateButton->SetToolTip(_("Blind guiding/calibration: this feature becomes available after successful mount calibration and after "
        "Guiding Assistant run. DEC backlash estimation is not required for blind guiding. Guiding Assistant must be repeated after the MERIDIAN FLIP!"));

    pBlindGuidingGroup->AddSpacer(10);
    pBlindGuidingGroup->Add(pBlindCalibrationRadioGroup);
    pBlindGuidingGroup->Add(m_autoBlindGuidingCheckBox, 0, wxLEFT | wxALIGN_LEFT, 5);
    pBlindGuidingGroup->AddSpacer(10);
    pBlindGuidingGroup->Add(pBlindGuidingTable);
    pBlindGuidingGroup->AddSpacer(10);
    pBlindGuidingGroup->Add(m_calibrateButton, 1, wxALIGN_CENTER_HORIZONTAL, 5);
    pBlindGuidingGroup->AddSpacer(10);
    pBlindGuidingGroup->Add(statusboxSizer);
    pBlindGuidingGroup->AddSpacer(10);

    // Buttons
    wxBoxSizer *ButtonSizer = new wxBoxSizer(wxHORIZONTAL);
    m_CloseButton = new wxButton(this, wxID_ANY, _("Close"));
    ButtonSizer->Add(m_CloseButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    m_AdvancedButton = new wxButton(this, wxID_ANY, pPlanet->GetShowAdvancedSettings() ? _("Advanced <<<") : _("Advanced >>>"));
    ButtonSizer->Add(m_AdvancedButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

    // Left size panel
    wxBoxSizer *leftSizer = new wxStaticBoxSizer(wxVERTICAL, this, _T("Basic settings"));
    leftSizer->AddSpacer(10);
    leftSizer->Add(m_enableCheckBox, 0, wxLEFT | wxALIGN_LEFT, 20);
    leftSizer->AddSpacer(10);
    leftSizer->Add(m_featureTrackingCheckBox, 0, wxLEFT | wxALIGN_LEFT, 20);
    leftSizer->AddSpacer(10);
    leftSizer->Add(m_tabs, 1, wxEXPAND | wxALL, 10);
    leftSizer->AddSpacer(10);
    leftSizer->Add(pCamGroup);

    // Right size panel
    wxBoxSizer* rightSizer = new wxStaticBoxSizer(wxVERTICAL, this, _T("Advanced settings"));
    rightSizer->AddSpacer(10);
    rightSizer->Add(m_ShowElements, 0, wxLEFT | wxALIGN_LEFT, 20);
    rightSizer->AddSpacer(10);
    rightSizer->Add(m_NoiseFilter, 0, wxLEFT | wxALIGN_LEFT, 20);
    rightSizer->AddSpacer(10);
    rightSizer->Add(m_saveVideoLogCheckBox, 0, wxLEFT | wxALIGN_LEFT, 20);
    rightSizer->AddSpacer(10);
    rightSizer->Add(pBlindGuidingGroup, 0, wxALL, 5);

    // Hide/show the advanced settings
    bool showAdvanced = pPlanet->GetShowAdvancedSettings();
    m_advancedSizer = rightSizer;
    m_advancedSizer->Show(showAdvanced);
    if (showAdvanced)
        m_advancedTimer.Start(3000);
    else
        m_advancedTimer.Stop();

    // Both left and right panels
    wxBoxSizer* mainSizer = new wxBoxSizer(wxHORIZONTAL);
    mainSizer->Add(leftSizer, 1, wxEXPAND | wxALL, 5);
    mainSizer->Add(rightSizer, 0, wxEXPAND | wxALL, 5);
    m_mainSizer = mainSizer;

    // All top level controls
    wxBoxSizer* topSizer = new wxBoxSizer(wxVERTICAL);
    topSizer->Add(mainSizer, 1, wxEXPAND | wxALL, 5);
    topSizer->Add(ButtonSizer, 0, wxALL | wxALIGN_CENTER_HORIZONTAL, 5);
    SetSizer(topSizer);
    Layout();
    topSizer->Fit(this);

    // Connect Events
    Bind(wxEVT_TIMER, &PlanetToolWin::OnAdvancedTimer, this, wxID_ANY);
    m_enableCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnEnableToggled, this);
    m_featureTrackingCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnSurfaceTrackingClick, this);
    m_CloseButton->Bind(wxEVT_COMMAND_BUTTON_CLICKED, &PlanetToolWin::OnCloseButton, this);
    m_CloseButton->Bind(wxEVT_KEY_DOWN, &PlanetToolWin::OnKeyDown, this);
    m_CloseButton->Bind(wxEVT_KEY_UP, &PlanetToolWin::OnKeyUp, this);
    m_CloseButton->Bind(wxEVT_ENTER_WINDOW, &PlanetToolWin::OnMouseEnterCloseBtn, this);
    m_CloseButton->Bind(wxEVT_LEAVE_WINDOW, &PlanetToolWin::OnMouseLeaveCloseBtn, this);
    m_AdvancedButton->Bind(wxEVT_COMMAND_BUTTON_CLICKED, &PlanetToolWin::OnAdvancedButton, this);
    m_RoiCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnRoiModeClick, this);
    Bind(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(PlanetToolWin::OnClose), this);

    m_minRadius->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_minRadius), NULL, this);
    m_maxRadius->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_maxRadius), NULL, this);

    pPlanet->SetPlanetaryElementsButtonState(false);
    pPlanet->SetPlanetaryElementsVisual(false);

    m_minRadius->SetValue(pPlanet->GetPlanetaryParam_minRadius());
    m_maxRadius->SetValue(pPlanet->GetPlanetaryParam_maxRadius());
    m_ThresholdSlider->SetValue(pPlanet->GetPlanetaryParam_highThreshold());
    m_minHessianSlider->SetValue(pPlanet->GetPlanetaryParam_minHessian());
    m_maxFeaturesSlider->SetValue(pPlanet->GetPlanetaryParam_maxFeatures());
    m_featureTrackingCheckBox->SetValue(pPlanet->GetSurfaceTrackingState());
    m_RoiCheckBox->SetValue(pPlanet->GetRoiEnableState());
    m_NoiseFilter->SetValue(pPlanet->GetNoiseFilterState());
    m_enableCheckBox->SetValue(pPlanet->GetPlanetaryEnableState());
    m_saveVideoLogCheckBox->SetValue(pPlanet->GetVideoLogging());
    m_manualCalibration->SetValue(pPlanet->GetBlindGuidingCalMode() == GuiderPlanet::BLIND_CAL_MANUAL);
    m_autoCalibration->SetValue(pPlanet->GetBlindGuidingCalMode() == GuiderPlanet::BLIND_CAL_AUTO);
    m_autoBlindGuidingCheckBox->SetValue(pPlanet->GetAutoBlindGuiding());
    m_driftRaGainCtrl->SetValue(pPlanet->GetDriftRaGain());
    m_driftDecGainCtrl->SetValue(pPlanet->GetDriftDecGain());

    // Update blind guiding status
    pPlanet->DetermineBlindCalibrationStatus();
    m_BlindGuidingStatus->SetLabel(pPlanet->GetBlindCalibrationStatus());

    SetEnabledState(this, pPlanet->GetPlanetaryEnableState());

    m_tabs->SetSelection(pPlanet->GetSurfaceTrackingState() ? 1 : 0);

    // Get the current camera settings
    int exposureMsec;
    bool auto_exp;
    pFrame->GetExposureInfo(&exposureMsec, &auto_exp);
    if (exposureMsec)
    {
        exposureMsec = wxMin(exposureMsec, 9999);
        m_ExposureCtrl->SetValue(exposureMsec);
        pFrame->SetExposureDuration(exposureMsec, true);
    }
    else
    {
        exposureMsec = pConfig->Profile.GetInt("/ExposureDurationMs", 1000);
    }
    m_DelayCtrl->SetValue(pFrame->GetTimeLapse());
    if (pCamera)
        m_GainCtrl->SetValue(pCamera->GetCameraGain());

    Connect(APPSTATE_NOTIFY_EVENT, wxCommandEventHandler(PlanetToolWin::OnAppStateNotify));

    int xpos = pConfig->Profile.GetInt("/PlanetTool/pos.x", -1);
    int ypos = pConfig->Profile.GetInt("/PlanetTool/pos.y", -1);
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
    // Stop the timer
    m_advancedTimer.Stop();

    pFrame->eventLock.Lock();
    pFrame->pPlanetTool = nullptr;
    pFrame->eventLock.Unlock();
}

void PlanetToolWin::OnEnableToggled(wxCommandEvent& event)
{
    GuiderMultiStar* pMultiGuider = dynamic_cast<GuiderMultiStar*>(pFrame->pGuider);

    if (m_enableCheckBox->IsChecked())
    {
        pFrame->SaveStarFindMode();
        pFrame->SetStarFindMode(Star::FIND_PLANET);
        pPlanet->SetPlanetaryEnableState(true);
        pFrame->m_PlanetaryMenuItem->Check(true);
        SetEnabledState(this, true);

        if (pMultiGuider)
        {
            // Save the current state of the mass change threshold and disable it
            bool prev = pMultiGuider->GetMassChangeThresholdEnabled();
            pMultiGuider->SetMassChangeThresholdEnabled(false);
            pConfig->Profile.SetBoolean("/guider/onestar/MassChangeThresholdEnabled", prev);
        }

        // Disable subframes
        if (pCamera)
        {
            pConfig->Profile.SetBoolean("/camera/UseSubframes", pCamera->UseSubframes);
            pCamera->UseSubframes = false;
        }

        // Disable multi-star mode
        bool prev = pFrame->pGuider->GetMultiStarMode();
        pFrame->pGuider->SetMultiStarMode(false);
        pConfig->Profile.SetBoolean("/guider/multistar/enabled", prev);

        Debug.Write(_("Planetary tracking: enabled\n"));
    }
    else
    {
        pFrame->RestoreStarFindMode();
        pPlanet->SetPlanetaryEnableState(false);
        pFrame->m_PlanetaryMenuItem->Check(false);
        SetEnabledState(this, false);

        // Restore the previous state of the mass change threshold
        if (pMultiGuider)
        {
            bool prev = pConfig->Profile.GetBoolean("/guider/onestar/MassChangeThresholdEnabled", false);
            pMultiGuider->SetMassChangeThresholdEnabled(prev);
        }

        // Restore subframes
        if (pCamera)
        {
            pCamera->UseSubframes = pConfig->Profile.GetBoolean("/camera/UseSubframes", false);
        }

        // Restore multi-star mode
        bool prev = pConfig->Profile.GetBoolean("/guider/multistar/enabled", false);
        pFrame->pGuider->SetMultiStarMode(prev);

        Debug.Write(_("Planetary tracking: disabled\n"));
    }

    // Update elements display state
    m_tabs->SetSelection(pPlanet->GetSurfaceTrackingState() ? 1 : 0);
    pFrame->pStatsWin->ClearPlanetStats();
    OnShowElementsClick(event);
}

// Toggle surface features detection/tracking mode
void PlanetToolWin::OnSurfaceTrackingClick(wxCommandEvent& event)
{
    bool featureTracking = m_featureTrackingCheckBox->IsChecked();
    pPlanet->SetSurfaceTrackingState(featureTracking);
    m_tabs->SetSelection(featureTracking ? 1 : 0);
    UpdateStatus();
    Debug.Write(wxString::Format("Planetary tracking: %s surface features mode\n", featureTracking ? "enabled" : "disabled"));
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

void PlanetToolWin::OnRoiModeClick(wxCommandEvent& event)
{
    bool enabled = m_RoiCheckBox->IsChecked();
    pPlanet->SetRoiEnableState(enabled);
    Debug.Write(wxString::Format("Planetary tracking: %s ROI\n", enabled ? "enabled" : "disabled"));
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

void PlanetToolWin::OnNoiseFilterClick(wxCommandEvent& event)
{
    bool enabled = m_NoiseFilter->IsChecked();
    pPlanet->SetNoiseFilterState(enabled);
    Debug.Write(wxString::Format("Planetary tracking: %s noise filter\n", enabled ? "enabled" : "disabled"));
}

void PlanetToolWin::OnSaveVideoLog(wxCommandEvent& event)
{
    bool enabled = m_saveVideoLogCheckBox->IsChecked();
    pPlanet->SetVideoLogging(enabled);
    Debug.Write(wxString::Format("Planetary tracking: %s video log\n", enabled ? "enabled" : "disabled"));
}

void PlanetToolWin::OnExposureChanged(wxSpinDoubleEvent& event)
{
    int expMsec = m_ExposureCtrl->GetValue();
    expMsec = wxMin(expMsec, 9999);
    expMsec = wxMax(expMsec, 1);
    pFrame->SetExposureDuration(expMsec, true);
}

void PlanetToolWin::OnDelayChanged(wxSpinDoubleEvent& event)
{
    int delayMsec = m_DelayCtrl->GetValue();
    delayMsec = wxMin(delayMsec, 60000);
    delayMsec = wxMax(delayMsec, 0);
    pFrame->SetTimeLapse(delayMsec);
}

void PlanetToolWin::OnGainChanged(wxSpinDoubleEvent& event)
{
    int gain = m_GainCtrl->GetValue();
    gain = wxMin(gain, 100.0);
    gain = wxMax(gain, 0.0);
    if (pCamera)
        pCamera->SetCameraGain(gain);
}

// Update the state of the blind guiding controls depending on the 
// current state of the mount and estimation of the drift rates.
void PlanetToolWin::UpdateBlindGuidingControls()
{
    bool enabled = pPlanet->GetPlanetaryEnableState();
    bool blindGuidingAllowed = pPlanet->IsDriftValid() &&
        (pMount && pMount->IsCalibrated() && pMount->IsConnected()) && pFrame->pGuider->IsGuiding();

    m_manualCalibration->Enable(enabled && blindGuidingAllowed);
    m_autoCalibration->Enable(enabled && blindGuidingAllowed);

    bool autodrift = pPlanet->GetBlindGuidingCalMode() == GuiderPlanet::BLIND_CAL_AUTO;
    m_driftRaGainCtrl->Enable(enabled && blindGuidingAllowed && !autodrift);
    m_driftDecGainCtrl->Enable(enabled && blindGuidingAllowed && !autodrift);
    m_autoBlindGuidingCheckBox->Enable(enabled && blindGuidingAllowed);

    // Forced blind guiding can become disabled when guiding stops
    if (autodrift)
    {
        // Update the blind guiding controls
        m_driftRaGainCtrl->SetValue(pPlanet->GetDriftRaGain());
        m_driftDecGainCtrl->SetValue(pPlanet->GetDriftDecGain());
    }

    // When blind guiding is active, the radio buttons are disabled
    m_manualCalibration->Enable(enabled && blindGuidingAllowed && !pPlanet->BlindGuidingRequested());
    m_autoCalibration->Enable(enabled && blindGuidingAllowed && !pPlanet->BlindGuidingRequested());

    // Update blind guiding status and control button
    pPlanet->DetermineBlindCalibrationStatus();
    m_BlindGuidingStatus->SetLabel(pPlanet->GetBlindCalibrationStatus());
    m_calibrateButton->Enable(enabled && blindGuidingAllowed);
    switch (pPlanet->GetBlindGuidingCalMode())
    {
    case GuiderPlanet::BLIND_CAL_AUTO:
    {
        if (pPlanet->GetBlindGuidingState())
            m_calibrateButton->SetLabel(_("Stop Calibration"));
        else
            m_calibrateButton->SetLabel(_("Start Automatic Calibration"));
        break;
    }
    case GuiderPlanet::BLIND_CAL_MANUAL:
    {
        if (pPlanet->GetBlindGuidingState())
            m_calibrateButton->SetLabel(_("Stop Blind Guiding"));
        else
            m_calibrateButton->SetLabel(_("Start Blind Guiding"));
        break;
    }
    }

    // Update calibration button size to match the label
    m_calibrateButton->GetParent()->Layout();
}

void PlanetToolWin::OnBlindGuidingButton(wxCommandEvent& event)
{
    bool enabled = pPlanet->GetPlanetaryEnableState();
    bool blindGuidingAllowed = pPlanet->IsDriftValid() &&
        (pMount && pMount->IsCalibrated() && pMount->IsConnected()) && pFrame->pGuider->IsGuiding();

    if (pPlanet->GetBlindGuidingCalMode() == GuiderPlanet::BLIND_CAL_AUTO)
    {
        if (!pPlanet->GetBlindGuidingState())
        {
            if (blindGuidingAllowed && enabled && !pPlanet->BlindGuidingRequested())
            {
                pPlanet->StartBlindGuidingCalibration();
                Debug.Write(_("Blind guiding: user request to start automatic drift gain calibration\n"));
            }
        }
        else
        {
            pPlanet->StopBlindGuidingCalibration();
            Debug.Write(_("Blind guiding: user request to stop automatic drift gain calibration\n"));
        }
    }
    else
    {
        if (!pPlanet->GetBlindGuidingState())
        {
            if (blindGuidingAllowed && enabled && !pPlanet->BlindGuidingRequested())
            {
                pPlanet->StartBlindGuiding();
                Debug.Write(_("Blind guiding: user request to start manual blind guiding\n"));
            }
        }
        else
        {
            pPlanet->StopBlindGuiding();
            Debug.Write(_("Blind guiding: user request to stop manual blind guiding\n"));
        }
    }
    UpdateBlindGuidingControls();
}

void PlanetToolWin::OnBlindGuidingRadioButton(wxCommandEvent& event)
{
    if (event.GetEventObject() == m_manualCalibration)
    {
        pPlanet->SetBlindGuidingCalMode(GuiderPlanet::BLIND_CAL_MANUAL);
        Debug.Write(_("Blind guiding: selected manual calibration/guiding"));
    }
    else if (event.GetEventObject() == m_autoCalibration)
    {
        pPlanet->SetBlindGuidingCalMode(GuiderPlanet::BLIND_CAL_AUTO);
        Debug.Write(_("Blind guiding: selected automatic calibration"));
    }
    UpdateBlindGuidingControls();
}

void PlanetToolWin::OnAutoBlindGuidingCheckBox(wxCommandEvent& event)
{
    bool enabled = m_autoBlindGuidingCheckBox->IsChecked();
    pPlanet->SetAutoBlindGuiding(enabled);
    Debug.Write(wxString::Format("Blind guiding: %s auto-activate on signal loss\n", enabled ? "enabled" : "disabled"));
}

void PlanetToolWin::OnDriftRaGainChanged(wxSpinDoubleEvent& event)
{
    double gain = m_driftRaGainCtrl->GetValue();
    gain = wxMin(gain, 2.0);
    gain = wxMax(gain, 0.0);
    pPlanet->SetDriftRaGain(gain);
}

void PlanetToolWin::OnDriftDecGainChanged(wxSpinDoubleEvent& event)
{
    double gain = m_driftDecGainCtrl->GetValue();
    gain = wxMin(gain, 2.0);
    gain = wxMax(gain, 0.0);
    pPlanet->SetDriftDecGain(gain);
}

void PlanetToolWin::UpdateStatus()
{
    bool enabled = pPlanet->GetPlanetaryEnableState();
    bool surfaceTracking = pPlanet->GetSurfaceTrackingState();

    // Update planetary tracking controls
    m_featureTrackingCheckBox->Enable(enabled);
    m_minRadius->Enable(enabled && !surfaceTracking);
    m_maxRadius->Enable(enabled && !surfaceTracking);
    m_RoiCheckBox->Enable(enabled && !surfaceTracking);
    m_ShowElements->Enable(enabled);
    m_NoiseFilter->Enable(enabled);
    m_saveVideoLogCheckBox->Enable(enabled);
    UpdateBlindGuidingControls();

    // Update slider states
    m_ThresholdSlider->Enable(enabled && !surfaceTracking);
    m_minHessianSlider->Enable(enabled && surfaceTracking);
    m_maxFeaturesSlider->Enable(enabled && surfaceTracking);

    // Update tabs state
    m_featuresTab->Enable(surfaceTracking);
    m_planetTab->Enable(!surfaceTracking);

    // Toggle the visibility of planetary stats grid
    pFrame->pStatsWin->ShowPlanetStats(enabled);

    // For use with simulator only
    pPlanet->m_cameraSimulationRefPointValid = false;
    pPlanet->m_simulationZeroOffset = true;
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
    int highThreshold = event.GetInt();
    highThreshold = wxMin(highThreshold, PT_HIGH_THRESHOLD_MAX);
    highThreshold = wxMax(highThreshold, PT_THRESHOLD_MIN);
    int lowThreshold = wxMax(highThreshold / 2, PT_THRESHOLD_MIN);
    pPlanet->SetPlanetaryParam_lowThreshold(lowThreshold);
    pPlanet->SetPlanetaryParam_highThreshold(highThreshold);
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
    pPlanet->SetPlanetaryElementsButtonState(false);
    pPlanet->SetPlanetaryElementsVisual(false);
    pFrame->pGuider->Refresh();
    pFrame->pGuider->Update();

    // save the window position
    int x, y;
    GetPosition(&x, &y);
    pConfig->Profile.SetInt("/PlanetTool/pos.x", x);
    pConfig->Profile.SetInt("/PlanetTool/pos.y", y);

    // Revert to a default duration of tooltip display (apparently 5 seconds)
    wxToolTip::SetAutoPop(5000);

    Destroy();
}

void PlanetToolWin::OnCloseButton(wxCommandEvent& event)
{
    // Reset all to defaults
    if (wxGetKeyState(WXK_ALT))
    {
        pPlanet->SetPlanetaryParam_minRadius(PT_MIN_RADIUS_DEFAULT);
        pPlanet->SetPlanetaryParam_maxRadius(PT_MAX_RADIUS_DEFAULT);
        pPlanet->SetPlanetaryParam_lowThreshold(PT_HIGH_THRESHOLD_DEFAULT/2);
        pPlanet->SetPlanetaryParam_highThreshold(PT_HIGH_THRESHOLD_DEFAULT);
        pPlanet->SetPlanetaryParam_minHessian(PT_MIN_HESSIAN_DEFAULT);
        pPlanet->SetPlanetaryParam_maxFeatures(PT_MAX_SURFACE_FEATURES);
        pPlanet->SetDriftRaGain(PT_BLIND_DRIFT_GAIN_DEFAULT);
        pPlanet->SetDriftDecGain(PT_BLIND_DRIFT_GAIN_DEFAULT);
        pPlanet->SetNoiseFilterState(false);
        pPlanet->SetBlindGuidingCalMode(GuiderPlanet::BLIND_CAL_MANUAL);
        pPlanet->SetAutoBlindGuiding(false);

        m_minRadius->SetValue(pPlanet->GetPlanetaryParam_minRadius());
        m_maxRadius->SetValue(pPlanet->GetPlanetaryParam_maxRadius());
        m_ThresholdSlider->SetValue(pPlanet->GetPlanetaryParam_highThreshold());
        m_minHessianSlider->SetValue(pPlanet->GetPlanetaryParam_minHessian());
        m_maxFeaturesSlider->SetValue(pPlanet->GetPlanetaryParam_maxFeatures());
        m_NoiseFilter->SetValue(pPlanet->GetNoiseFilterState());
        m_driftRaGainCtrl->SetValue(pPlanet->GetDriftRaGain());
        m_driftDecGainCtrl->SetValue(pPlanet->GetDriftDecGain());
        m_manualCalibration->SetValue(pPlanet->GetBlindGuidingCalMode() == GuiderPlanet::BLIND_CAL_MANUAL);
        m_autoCalibration->SetValue(pPlanet->GetBlindGuidingCalMode() == GuiderPlanet::BLIND_CAL_AUTO);
        m_autoBlindGuidingCheckBox->SetValue(pPlanet->GetAutoBlindGuiding());
    }
    else
        this->Close();
}

void PlanetToolWin::OnAdvancedButton(wxCommandEvent& event)
{
    bool show = !m_mainSizer->IsShown(m_advancedSizer);
    m_AdvancedButton->SetLabel(show ? _("Advanced <<<") : _("Advanced >>>"));
    m_advancedSizer->Show(show);
    Layout();
    GetSizer()->Fit(this);
    pPlanet->SetShowAdvancedSettings(show);

    // Call timer every few seconds when advanced settings are shown
    if (show)
        m_advancedTimer.Start(3000);
    else
        m_advancedTimer.Stop();
}

// Sync local camera settings with the main frame changes
void PlanetToolWin::OnAppStateNotify(wxCommandEvent& event)
{
    int exposureMsec;
    bool auto_exp;
    pFrame->GetExposureInfo(&exposureMsec, &auto_exp);
    if (exposureMsec)
    {
        exposureMsec = wxMin(exposureMsec, 9999);
        if (exposureMsec != m_ExposureCtrl->GetValue())
            m_ExposureCtrl->SetValue(exposureMsec);
    }

    int const delayMsec = pFrame->GetTimeLapse();
    if (delayMsec != m_DelayCtrl->GetValue())
        m_DelayCtrl->SetValue(delayMsec);

    if (pCamera)
    {
        int const gain = pCamera->GetCameraGain();
        if (gain != m_GainCtrl->GetValue())
            m_GainCtrl->SetValue(gain);
    }
}

// Called once in a while to update the UI controls
void PlanetToolWin::OnAdvancedTimer(wxTimerEvent& event)
{
    UpdateBlindGuidingControls();
}

wxWindow *PlanetTool::CreatePlanetToolWindow()
{
    return new PlanetToolWin();
}