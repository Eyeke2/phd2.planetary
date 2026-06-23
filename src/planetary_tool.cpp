/*
 *  planetary_tool.cpp
 *  PHD Guiding
 *
 *  Created by Leo Shatz.
 *  Copyright (c) 2023-2026 Leo Shatz
 *  Copyright (c) 2018-2026 openphdguiding.org
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

#include <wx/tooltip.h>

static bool pauseAlert = false;
static wxString planetaryPauseAlertMsg =
    _("Planetary detection paused : do not stop guiding to keep the original lock position!");
static constexpr double SIDEREAL_RATE_ARCSEC_PER_SEC = 15.041;
static constexpr double MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR = 5.0 * SIDEREAL_RATE_ARCSEC_PER_SEC * 3600.0;
static constexpr double MAX_MOUNT_TRACK_RATE_OFFSET = 10.0 * 3600.0 / (15.0 * 0.9973);

struct PlanetToolWin : public wxDialog
{
    SolarSystemObject *pSolarSystemObj;

    wxTimer m_planetaryTimer;

    bool m_panelVisible;
    wxPanel *m_planetPanel;
    wxCheckBox *m_enableCheckBox;
    wxCheckBox *m_saveVideoLogCheckBox;

    wxSpinCtrlDouble *m_minRadius;
    wxSpinCtrlDouble *m_maxRadius;

    wxSlider *m_thresholdSlider;

    // Controls for camera settings, duplicating the ones from camera setup dialog and exposure time dropdown.
    // Used for streamlining the solar/planetary mode guiding user experience.
    wxSpinCtrlDouble *m_ExposureCtrl;
    wxSpinCtrlDouble *m_DelayCtrl;
    wxSpinCtrlDouble *m_GainCtrl;
    wxChoice *m_BinningCtrl;

    // Mount controls
    enum DriveRates m_driveRate;
    wxChoice *m_mountGuidingRate;
    wxCheckBox *m_mountTrackigCheckBox;
    Scope *m_prevPointingSource;
    bool m_prevMountConnected;
    bool m_prevMountCustomRateNonZero;

    // Custom rate controls
    wxStaticBoxSizer *m_mountGroup;
    wxStaticBoxSizer *m_customRatesGroup;
    wxCheckBox *m_overrideCustomRate;
    wxRadioButton *m_customMountTrackingMode;
    wxRadioButton *m_minorBodyTrackingMode;
    wxStaticText *m_customRateRaLabel;
    wxStaticText *m_customRateDecLabel;
    wxSpinCtrlDouble *m_Horizons_dRaCosDRate;
    wxSpinCtrlDouble *m_Horizons_dDecRate;
    wxButton *m_applyCustomRateButton;
    wxButton *m_minorBodyTrackingButton;
    wxButton *m_stopMinorBodyTrackingButton;
    bool m_updatingCustomRateFields;

    wxButton *m_CloseButton;
    wxButton *m_PauseButton;
    wxCheckBox *m_RoiCheckBox;
    wxCheckBox *m_ShowElements;
#ifdef DEVELOPER_MODE
    wxCheckBox *m_NoiseFilter;
#endif
    bool m_MouseHoverFlag;

    PlanetToolWin();
    ~PlanetToolWin();

    void OnAppStateNotify(wxCommandEvent& event);
    void OnPlanetaryTimer(wxTimerEvent& event);
    void OnPauseButton(wxCommandEvent& event);
    void OnClose(wxCloseEvent& event);
    void OnCloseButton(wxCommandEvent& event);
    void OnKeyDown(wxKeyEvent& event);
    void OnKeyUp(wxKeyEvent& event);
    void OnMouseEnterCloseBtn(wxMouseEvent& event);
    void OnMouseLeaveCloseBtn(wxMouseEvent& event);
    void OnThresholdChanged(wxCommandEvent& event);

    void OnEnableToggled(wxCommandEvent& event);
    void OnSpinCtrl_minRadius(wxSpinDoubleEvent& event);
    void OnSpinCtrl_maxRadius(wxSpinDoubleEvent& event);
    void OnRoiModeClick(wxCommandEvent& event);
    void OnShowElementsClick(wxCommandEvent& event);
#ifdef DEVELOPER_MODE
    void OnNoiseFilterClick(wxCommandEvent& event);
#endif
    void OnMountTrackingClick(wxCommandEvent& event);
    void OnOverrideCustomRateClick(wxCommandEvent& event);
    void OnMountTrackingRateClick(wxCommandEvent& event);
    void OnTrackingRateMouseWheel(wxMouseEvent& event);
    void OnMinorBodyTrackingModeClick(wxCommandEvent& event);
    void OnHorizonsRateChanged(wxSpinDoubleEvent& event);
    void OnCustomRateChar(wxKeyEvent& event);
    void OnApplyCustomRateClick(wxCommandEvent& event);
    void OnMinorBodyTrackingClick(wxCommandEvent& event);
    void OnStopMinorBodyTrackingClick(wxCommandEvent& event);

    void OnExposureChanged(wxSpinDoubleEvent& event);
    void OnDelayChanged(wxSpinDoubleEvent& event);
    void OnGainChanged(wxSpinDoubleEvent& event);
    void OnBinningSelected(wxCommandEvent& event);
    void OnSaveVideoLog(wxCommandEvent& event);

    bool CanApplyCustomMountRates() const;
    bool CanEditCustomRates() const;
    void UpdateCustomRateLabels();
    void UpdateCustomRateRanges();
    void UpdateCustomRateControlsEnabled();
    void SetCustomRateFieldHighlight(bool applied);
    void ApplyCurrentCustomRate();
    bool GetCurrentCustomRateMountOffsets(double *raOffset, double *decOffset) const;
    bool GetCurrentCustomRateShiftRates(double *raRate, double *decRate) const;
    bool MinorBodyTrackingActive() const;
    void RefreshCustomRateShiftReadback(double declination, double raRate, double decRate);
    void RefreshCustomRateReadback(double declination, double raRate, double decRate);
    void SyncCameraExposure(bool init = false);
    void CheckMinExposureDuration();
    void UpdateStatus();
};

static wxString TITLE = wxTRANSLATE("Planetary and solar guiding | disabled");
static wxString TITLE_ACTIVE = wxTRANSLATE("Planetary and solar guiding | enabled");
static wxString TITLE_PAUSED = wxTRANSLATE("Planetary and solar guiding | paused");

static void SetEnabledState(PlanetToolWin *win, bool active)
{
    bool paused = win->pSolarSystemObj->GetDetectionPausedState();
    win->SetTitle(wxGetTranslation(active ? (paused ? TITLE_PAUSED : TITLE_ACTIVE) : TITLE));
    win->UpdateStatus();
}

// Utility function to add the <label, input> pairs to a flexgrid
static void AddTableEntryPair(wxWindow *parent, wxFlexGridSizer *pTable, const wxString& label, wxWindow *pControl,
                              const wxString& tooltip)
{
    wxStaticText *pLabel = new wxStaticText(parent, wxID_ANY, label + _(": "), wxPoint(-1, -1), wxSize(-1, -1));
    pLabel->SetToolTip(tooltip);
    pTable->Add(pLabel, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    pTable->Add(pControl, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
}

// Utility function to add the <label, input> pairs to a boxsizer
static void AddTableEntryPair(wxWindow *parent, wxBoxSizer *pSizer, const wxString& label, int spacer1, wxWindow *pControl,
                              int spacer2, const wxString& tooltip)
{
    wxStaticText *pLabel = new wxStaticText(parent, wxID_ANY, label + _(": "), wxPoint(-1, -1), wxSize(-1, -1));
    pLabel->SetToolTip(tooltip);
    pSizer->Add(pLabel, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 10);
    pSizer->AddSpacer(spacer1);
    pSizer->Add(pControl, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 10);
    pSizer->AddSpacer(spacer2);
}

static wxSpinCtrlDouble *NewSpinner(wxWindow *parent, wxString formatstr, double val, double minval, double maxval, double inc)
{
    wxSize sz = pFrame->GetTextExtent(wxString::Format(formatstr, maxval));
    wxSpinCtrlDouble *pNewCtrl = pFrame->MakeSpinCtrlDouble(parent, wxID_ANY, wxEmptyString, wxDefaultPosition, sz,
                                                            wxSP_ARROW_KEYS, minval, maxval, val, inc);
    pNewCtrl->SetDigits(0);
    return pNewCtrl;
}

static bool HorizonsRatesToMountOffsets(double decRadians, double horizonsRaCosDRate, double horizonsDecRate,
                                        double *raOffset, double *decOffset)
{
    if (!raOffset || !decOffset || decRadians == UNKNOWN_DECLINATION)
        return false;

    double const cosDec = cos(decRadians);
    if (fabs(cosDec) < 1e-4)
        return false;

    *raOffset = horizonsRaCosDRate / (15.0 * 3600.0 * cosDec);
    *decOffset = horizonsDecRate / 3600.0;
    if (fabs(*raOffset) > MAX_MOUNT_TRACK_RATE_OFFSET || fabs(*decOffset) > MAX_MOUNT_TRACK_RATE_OFFSET)
        return false;
    return true;
}

static bool MountOffsetsToHorizonsRates(double decRadians, double raOffset, double decOffset,
                                        double *horizonsRaCosDRate, double *horizonsDecRate)
{
    if (!horizonsRaCosDRate || !horizonsDecRate || decRadians == UNKNOWN_DECLINATION)
        return false;

    *horizonsRaCosDRate = raOffset * 15.0 * cos(decRadians) * 3600.0;
    *horizonsDecRate = decOffset * 3600.0;
    return true;
}

static bool HorizonsRatesToShiftRates(double decRadians, double horizonsRaCosDRate, double horizonsDecRate,
                                      double *raRate, double *decRate)
{
    if (!raRate || !decRate || decRadians == UNKNOWN_DECLINATION)
        return false;

    double const cosDec = cos(decRadians);
    if (fabs(cosDec) < 1e-4)
        return false;

    *raRate = horizonsRaCosDRate / cosDec;
    *decRate = horizonsDecRate;
    if (fabs(*raRate) > MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR || fabs(*decRate) > MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR)
        return false;
    return true;
}

static bool ShiftRatesToHorizonsRates(double decRadians, double raRate, double decRate,
                                      double *horizonsRaCosDRate, double *horizonsDecRate)
{
    if (!horizonsRaCosDRate || !horizonsDecRate || decRadians == UNKNOWN_DECLINATION)
        return false;

    *horizonsRaCosDRate = raRate * cos(decRadians);
    *horizonsDecRate = decRate;
    return true;
}

static bool IsCustomRateInputKey(int keyCode)
{
    return (keyCode >= '0' && keyCode <= '9') || keyCode == '.' || keyCode == '+' || keyCode == '-';
}

PlanetToolWin::PlanetToolWin()
    : wxDialog(pFrame, wxID_ANY, wxGetTranslation(TITLE), wxDefaultPosition, wxDefaultSize, wxCAPTION | wxCLOSE_BOX),
      m_planetaryTimer(this), pSolarSystemObj(&pFrame->pGuider->m_SolarSystemObject), m_MouseHoverFlag(false)

{
    SetSizeHints(wxDefaultSize, wxDefaultSize);

    // Set custom duration of tooltip display to 10 seconds
    wxToolTip::SetAutoPop(10000);

    m_panelVisible = true;
    m_planetPanel = new wxPanel(this, wxID_ANY);
    m_enableCheckBox = new wxCheckBox(this, wxID_ANY, _("Enable planetary and solar guiding"));
    m_enableCheckBox->SetToolTip(_("Toggle between star and solar/lunar/planetary guiding modes"));

#ifdef DEVELOPER_MODE
    // Experimental noise filter
    m_NoiseFilter = new wxCheckBox(this, wxID_ANY, _("Enable noise suppression filter (experimental)"));
    m_NoiseFilter->SetToolTip(_("Enable noise filtering only for extremely noisy images. Use this option cautiously, as it's "
                                "recommended only when absolutely necessary."));
#endif

    wxString radiusTooltip = _("For initial guess of possible radius range connect the gear and set correct focal length.");
    if (pCamera)
    {
        // arcsec/pixel
        double pixelScale = pFrame->GetPixelScale(pCamera->GetCameraPixelSize(), pFrame->GetFocalLength(), pCamera->GetBinning());
        if ((pFrame->GetFocalLength() > 1) && pixelScale > 0)
        {
            double radiusGuessMax = 990.0 / pixelScale;
            double raduisGuessMin = 870.0 / pixelScale;
            radiusTooltip = wxString::Format(_("Hint: for solar/lunar detection (pixel size=%.2f, binning=x%d, FL=%d mm) set "
                                               "the radius to approximately %.0f-%.0f."),
                                             pCamera->GetCameraPixelSize(), pCamera->GetBinning(), pFrame->GetFocalLength(),
                                             raduisGuessMin - 10, radiusGuessMax + 10);
        }
    }

    wxSize spinSize = wxSize(StringWidth(this, _T("0000")), -1);
    wxStaticText *minRadius_Label = new wxStaticText(m_planetPanel, wxID_ANY, _("Min radius:"));
    m_minRadius = pFrame->MakeSpinCtrlDouble(m_planetPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, spinSize,
                                             wxSP_ARROW_KEYS, PT_RADIUS_MIN, PT_RADIUS_MAX, PT_MIN_RADIUS_DEFAULT);
    minRadius_Label->SetToolTip(
        _("Minimum planet radius (in pixels). Set this a few pixels lower than the actual planet radius. ") + radiusTooltip);

    wxStaticText *maxRadius_Label = new wxStaticText(m_planetPanel, wxID_ANY, _("Max radius:"));
    m_maxRadius = pFrame->MakeSpinCtrlDouble(m_planetPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, spinSize,
                                             wxSP_ARROW_KEYS, PT_RADIUS_MIN, PT_RADIUS_MAX, PT_MAX_RADIUS_DEFAULT);
    maxRadius_Label->SetToolTip(
        _("Maximum planet radius (in pixels). Set this a few pixels higher than the actual planet radius. ") + radiusTooltip);

    wxBoxSizer *x_radii = new wxBoxSizer(wxHORIZONTAL);
    x_radii->Add(0, 0, 1, wxEXPAND, 5);
    x_radii->Add(minRadius_Label, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    x_radii->Add(m_minRadius, 0, wxALIGN_CENTER_VERTICAL, 5);
    x_radii->Add(0, 0, 1, wxEXPAND, 5);
    x_radii->Add(maxRadius_Label, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    x_radii->Add(m_maxRadius, 0, wxALIGN_CENTER_VERTICAL, 5);
    x_radii->Add(0, 0, 1, wxEXPAND, 5);

    // Planetary disk detection stuff
    wxStaticText *ThresholdLabel =
        new wxStaticText(m_planetPanel, wxID_ANY, _("Edge Detection Threshold:"), wxDefaultPosition, wxDefaultSize, 0);
    m_thresholdSlider = new wxSlider(m_planetPanel, wxID_ANY, PT_HIGH_THRESHOLD_DEFAULT, PT_THRESHOLD_MIN, PT_HIGH_THRESHOLD_MAX,
                                     wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL | wxSL_LABELS);
    ThresholdLabel->SetToolTip(_("Higher values reduce sensitivity to weaker edges, resulting in cleaner contour. This is "
                                 "displayed in red when the display of internal contour edges is enabled."));
    m_thresholdSlider->Bind(wxEVT_SLIDER, &PlanetToolWin::OnThresholdChanged, this);
    m_RoiCheckBox = new wxCheckBox(m_planetPanel, wxID_ANY, _("Enable ROI"));
    m_RoiCheckBox->SetToolTip(
        _("Enable automatically selected Region Of Interest (ROI) for improved processing speed and reduced CPU usage."));

    // Show/hide detected elements
    m_ShowElements = new wxCheckBox(m_planetPanel, wxID_ANY, _("Display internal edges"));
    m_ShowElements->SetToolTip(_("Toggle the visibility of internally detected edges and tune detection parameters "
                                 "to maintain a manageable number of these features while keeping them as close as possible to "
                                 "the object limb in the solar system object guiding mode."));

    // Add all solar system object tab elements
    wxStaticBoxSizer *planetSizer = new wxStaticBoxSizer(new wxStaticBox(m_planetPanel, wxID_ANY, _T("Detection Settings")), wxVERTICAL);
    planetSizer->AddSpacer(10);
    planetSizer->Add(m_RoiCheckBox, 0, wxLEFT | wxALIGN_LEFT, 10);
    planetSizer->AddSpacer(10);
    planetSizer->Add(m_ShowElements, 0, wxLEFT | wxALIGN_LEFT, 10);
    planetSizer->AddSpacer(20);
    planetSizer->Add(x_radii, 0, wxEXPAND, 5);
    planetSizer->AddSpacer(10);
    planetSizer->Add(ThresholdLabel, 0, wxLEFT | wxTOP, 10);
    planetSizer->Add(m_thresholdSlider, 1, wxALL | wxEXPAND, 10);
    m_planetPanel->SetSizer(planetSizer);
    m_planetPanel->Layout();

    // Mount settings group
    m_mountGroup = new wxStaticBoxSizer(wxVERTICAL, this, _("Mount settings"));
    wxFlexGridSizer *pMountTable = new wxFlexGridSizer(1, 6, 10, 10);
    m_mountTrackigCheckBox = new wxCheckBox(this, wxID_ANY, _("Tracking"));
    m_mountTrackigCheckBox->SetToolTip(_("Press and hold CTRL key to toggle mount tracking state"));
    wxArrayString rates;
    rates.Add(_("Sidereal"));
    m_mountGuidingRate = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, rates);
    m_mountGuidingRate->SetSelection(0);
    m_mountTrackigCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnMountTrackingClick, this);
    m_mountGuidingRate->Bind(wxEVT_CHOICE, &PlanetToolWin::OnMountTrackingRateClick, this);
    m_mountGuidingRate->Bind(wxEVT_MOUSEWHEEL, &PlanetToolWin::OnTrackingRateMouseWheel, this);

    pMountTable->Add(m_mountTrackigCheckBox, 0, wxALL | wxALIGN_CENTER_VERTICAL, 10);
    AddTableEntryPair(this, pMountTable, _("Tracking rate"), m_mountGuidingRate,
                      _("Select the desired tracking rate for the mount"));
    m_mountGroup->Add(pMountTable);

    ///////////////////////
    m_overrideCustomRate = new wxCheckBox(this, wxID_ANY, _("Enable custom tracking rates"));
    m_overrideCustomRate->SetToolTip(_("Enable custom tracking rates for the Custom tracking mode"));
    m_customRatesGroup = new wxStaticBoxSizer(wxVERTICAL, this, _("Custom rates"));
    wxBoxSizer *trackingModeSizer = new wxBoxSizer(wxVERTICAL);
    m_customMountTrackingMode = new wxRadioButton(this, wxID_ANY, _("Apply custom mount track rates"), wxDefaultPosition,
                                                  wxDefaultSize, wxRB_GROUP);
    m_customMountTrackingMode->SetToolTip(
        _("Apply the rates below directly to the mount RA and Dec tracking rates (relative to sidereal). "
          "This mode requires mount support for custom tracking rates via the ASCOM interface. "
          "Use when guiding directly on large non stellar targets such as the Sun, Moon, or planets. "
          "When guiding on stars, select the Comet/Minor Body tracking mode instead."));
    m_customMountTrackingMode->SetValue(true);
    m_minorBodyTrackingMode = new wxRadioButton(this, wxID_ANY, _("Comet/Minor Body tracking mode"));
    m_minorBodyTrackingMode->SetToolTip(
        _("Use the rates below when guiding on stars while tracking minor body motion. "
          "This option is available only when guiding is active. "
          "This mode keeps the comet nucleus or minor body fixed in the frame while stars are naturally elongated."));
    trackingModeSizer->Add(m_customMountTrackingMode, 0, wxBOTTOM | wxALIGN_CENTER_VERTICAL, 4);
    trackingModeSizer->Add(m_minorBodyTrackingMode, 0, wxALIGN_CENTER_VERTICAL);
    wxFlexGridSizer *HorizonsGrid = new wxFlexGridSizer(2, 2, 6, 25);
    m_customRateRaLabel = new wxStaticText(this, wxID_ANY, wxEmptyString);
    HorizonsGrid->Add(m_customRateRaLabel, 0, wxALIGN_CENTER_VERTICAL);
    wxSize horizonsSize = wxSize(StringWidth(this, _T("-0000.000000")), -1);
    m_Horizons_dRaCosDRate = pFrame->MakeSpinCtrlDouble(this, wxID_ANY, wxEmptyString, wxDefaultPosition, horizonsSize,
                                                        wxSP_ARROW_KEYS, -MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR,
                                                        MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR, 0.0, 1.0);
    m_Horizons_dRaCosDRate->SetDigits(6);
    m_Horizons_dRaCosDRate->Enable(false);
    HorizonsGrid->Add(m_Horizons_dRaCosDRate, 0, wxALIGN_CENTER_VERTICAL);

    m_customRateDecLabel = new wxStaticText(this, wxID_ANY, wxEmptyString);
    HorizonsGrid->Add(m_customRateDecLabel, 0, wxALIGN_CENTER_VERTICAL);
    m_Horizons_dDecRate = pFrame->MakeSpinCtrlDouble(this, wxID_ANY, wxEmptyString, wxDefaultPosition, horizonsSize,
                                                     wxSP_ARROW_KEYS, -MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR,
                                                     MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR, 0.0, 1.0);
    m_Horizons_dDecRate->SetDigits(6);
    m_Horizons_dDecRate->Enable(false);
    HorizonsGrid->Add(m_Horizons_dDecRate, 0, wxALIGN_CENTER_VERTICAL);
    m_applyCustomRateButton = new wxButton(this, wxID_ANY, _("Apply custom rate"));
    m_applyCustomRateButton->SetToolTip(_("Apply the custom mount tracking rates shown above when Tracking rate is set to Custom."));
    m_applyCustomRateButton->Enable(false);
    m_minorBodyTrackingButton = new wxButton(this, wxID_ANY, _("Start minor body tracking"));
    m_minorBodyTrackingButton->SetToolTip(_("Track a comet, asteroid, or other minor body while guiding on stars. Uses the "
                                           "rates below without changing the mount tracking rate."));
    m_minorBodyTrackingButton->Enable(false);
    m_stopMinorBodyTrackingButton = new wxButton(this, wxID_ANY, _("Stop minor body tracking"));
    m_stopMinorBodyTrackingButton->SetToolTip(
        _("Stop minor body tracking started here or from the Comet Tracking tool."));
    m_stopMinorBodyTrackingButton->Enable(false);

    m_mountGroup->Add(m_overrideCustomRate, 0, wxALL | wxALIGN_CENTER_VERTICAL, 10);
    m_customRatesGroup->Add(trackingModeSizer, 0, wxALL | wxALIGN_CENTER_VERTICAL, 10);
    m_customRatesGroup->Add(HorizonsGrid, 0, wxLEFT | wxRIGHT | wxBOTTOM | wxALIGN_LEFT, 10);
    m_customRatesGroup->Add(m_applyCustomRateButton, 0, wxLEFT | wxRIGHT | wxTOP | wxBOTTOM | wxEXPAND, 10);
    m_customRatesGroup->Add(m_minorBodyTrackingButton, 0, wxLEFT | wxRIGHT | wxBOTTOM | wxEXPAND, 10);
    m_customRatesGroup->Add(m_stopMinorBodyTrackingButton, 0, wxLEFT | wxRIGHT | wxBOTTOM | wxEXPAND, 10);
    m_mountGroup->Add(m_customRatesGroup, 0, wxLEFT | wxRIGHT | wxBOTTOM | wxEXPAND, 10);
    ///////////////////////

    // Camera settings group
    wxStaticBoxSizer *pCamGroup = new wxStaticBoxSizer(wxVERTICAL, this, _("Camera settings"));
    wxBoxSizer *pCamSizer1 = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer *pCamSizer2 = new wxBoxSizer(wxHORIZONTAL);
    m_ExposureCtrl = NewSpinner(this, _T("%5.0f"), 1000, PT_CAMERA_EXPOSURE_MIN, PT_CAMERA_EXPOSURE_MAX, 1);
    m_GainCtrl = NewSpinner(this, _T("%3.0f"), 0, 0, 100, 1);
    m_DelayCtrl = NewSpinner(this, _T("%5.0f"), 100, 0, 60000, 100);
    int maxBinning = pCamera ? (pCamera->Name == "Simulator" ? 1 : pCamera->MaxHwBinning) : 1;
    wxArrayString binningOpts;
    bool includeSwBinning = false; // TODO: SW binning UI
    GuideCamera::GetBinningOpts(&binningOpts, maxBinning, includeSwBinning);
    m_BinningCtrl = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, binningOpts);
    m_saveVideoLogCheckBox = new wxCheckBox(this, wxID_ANY, _("Enable video log"));
    m_saveVideoLogCheckBox->SetToolTip(_("Enable recording camera frames to video log file during guiding (using SER format)"));
    m_ExposureCtrl->Bind(wxEVT_SPINCTRLDOUBLE, &PlanetToolWin::OnExposureChanged, this);
    m_saveVideoLogCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnSaveVideoLog, this);
    m_GainCtrl->Bind(wxEVT_SPINCTRLDOUBLE, &PlanetToolWin::OnGainChanged, this);
    m_DelayCtrl->Bind(wxEVT_SPINCTRLDOUBLE, &PlanetToolWin::OnDelayChanged, this);
    m_BinningCtrl->Bind(wxEVT_CHOICE, &PlanetToolWin::OnBinningSelected, this);
    m_overrideCustomRate->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnOverrideCustomRateClick, this);
    m_customMountTrackingMode->Bind(wxEVT_RADIOBUTTON, &PlanetToolWin::OnMinorBodyTrackingModeClick, this);
    m_minorBodyTrackingMode->Bind(wxEVT_RADIOBUTTON, &PlanetToolWin::OnMinorBodyTrackingModeClick, this);
    m_Horizons_dRaCosDRate->Bind(wxEVT_SPINCTRLDOUBLE, &PlanetToolWin::OnHorizonsRateChanged, this);
    m_Horizons_dDecRate->Bind(wxEVT_SPINCTRLDOUBLE, &PlanetToolWin::OnHorizonsRateChanged, this);
    m_Horizons_dRaCosDRate->Bind(wxEVT_CHAR, &PlanetToolWin::OnCustomRateChar, this);
    m_Horizons_dDecRate->Bind(wxEVT_CHAR, &PlanetToolWin::OnCustomRateChar, this);
    m_applyCustomRateButton->Bind(wxEVT_BUTTON, &PlanetToolWin::OnApplyCustomRateClick, this);
    m_minorBodyTrackingButton->Bind(wxEVT_BUTTON, &PlanetToolWin::OnMinorBodyTrackingClick, this);
    m_stopMinorBodyTrackingButton->Bind(wxEVT_BUTTON, &PlanetToolWin::OnStopMinorBodyTrackingClick, this);
    pCamSizer1->AddSpacer(5);
    AddTableEntryPair(this, pCamSizer1, _("Exposure (ms)"), 20, m_ExposureCtrl, 20, _("Camera exposure in milliseconds)"));
    AddTableEntryPair(this, pCamSizer1, _("Gain"), 35, m_GainCtrl, 0, _("Camera gain (0-100)"));
    pCamSizer2->AddSpacer(5);
    AddTableEntryPair(this, pCamSizer2, _("Time Lapse (ms)"), 5, m_DelayCtrl, 20,
                      _("How long should PHD wait between guide frames? Useful when using very short exposures but wanting to "
                        "send guide commands less frequently"));
    AddTableEntryPair(this, pCamSizer2, _("Binning"), 10, m_BinningCtrl, 0,
                      _("Camera binning. For solar/planetary guiding 1x1 is recommended."));
    pCamGroup->Add(pCamSizer1);
    pCamGroup->AddSpacer(10);
    pCamGroup->Add(pCamSizer2);
    pCamGroup->AddSpacer(10);
    pCamGroup->Add(m_saveVideoLogCheckBox, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

    // Buttons
    wxBoxSizer *ButtonSizer = new wxBoxSizer(wxHORIZONTAL);
    m_CloseButton = new wxButton(this, wxID_ANY, _("Ok"));
    m_PauseButton = new wxButton(this, wxID_ANY, _("Pause"));
    m_PauseButton->SetToolTip(
        _("Use this button to pause/resume detection during clouds or totality instead of stopping guiding. "
          "It preserves object lock position, allowing PHD2 to realign the object without losing its original position"));
    ButtonSizer->Add(m_CloseButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    ButtonSizer->AddSpacer(15);
    ButtonSizer->Add(m_PauseButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

    // All top level controls
    wxBoxSizer *topSizer = new wxBoxSizer(wxVERTICAL);
    topSizer->AddSpacer(10);
    topSizer->Add(m_enableCheckBox, 0, wxLEFT | wxALIGN_LEFT, 20);
    topSizer->AddSpacer(10);
#ifdef DEVELOPER_MODE
    topSizer->Add(m_NoiseFilter, 0, wxLEFT | wxALIGN_LEFT, 20);
    topSizer->AddSpacer(10);
#endif
    topSizer->Add(m_planetPanel, 0, wxEXPAND | wxALL, 5);
    topSizer->AddSpacer(10);
    topSizer->Add(m_mountGroup, 0, wxEXPAND | wxALL, 5);
    topSizer->Add(pCamGroup, 0, wxEXPAND | wxALL, 5);
    topSizer->Add(ButtonSizer, 0, wxALL | wxALIGN_CENTER_HORIZONTAL, 5);

    SetSizer(topSizer);
    Layout();
    topSizer->Fit(this);

    // Connect Events
    Bind(wxEVT_TIMER, &PlanetToolWin::OnPlanetaryTimer, this, wxID_ANY);
    m_enableCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnEnableToggled, this);
    m_CloseButton->Bind(wxEVT_COMMAND_BUTTON_CLICKED, &PlanetToolWin::OnCloseButton, this);
    m_CloseButton->Bind(wxEVT_KEY_DOWN, &PlanetToolWin::OnKeyDown, this);
    m_CloseButton->Bind(wxEVT_KEY_UP, &PlanetToolWin::OnKeyUp, this);
    m_CloseButton->Bind(wxEVT_ENTER_WINDOW, &PlanetToolWin::OnMouseEnterCloseBtn, this);
    m_CloseButton->Bind(wxEVT_LEAVE_WINDOW, &PlanetToolWin::OnMouseLeaveCloseBtn, this);
    m_PauseButton->Bind(wxEVT_COMMAND_BUTTON_CLICKED, &PlanetToolWin::OnPauseButton, this);
    m_RoiCheckBox->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnRoiModeClick, this);
    m_ShowElements->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnShowElementsClick, this);
#ifdef DEVELOPER_MODE
    m_NoiseFilter->Bind(wxEVT_CHECKBOX, &PlanetToolWin::OnNoiseFilterClick, this);
#endif
    Bind(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(PlanetToolWin::OnClose), this);

    m_minRadius->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_minRadius), NULL, this);
    m_maxRadius->Connect(wxEVT_SPINCTRLDOUBLE, wxSpinDoubleEventHandler(PlanetToolWin::OnSpinCtrl_maxRadius), NULL, this);

    pSolarSystemObj->SetShowFeaturesButtonState(false);
    pSolarSystemObj->ShowVisualElements(false);

    m_minRadius->SetValue(pSolarSystemObj->Get_minRadius());
    m_maxRadius->SetValue(pSolarSystemObj->Get_maxRadius());
    m_thresholdSlider->SetValue(pSolarSystemObj->Get_highThreshold());
    m_RoiCheckBox->SetValue(pSolarSystemObj->GetRoiEnableState());
#ifdef DEVELOPER_MODE
    m_NoiseFilter->SetValue(pSolarSystemObj->GetNoiseFilterState());
#endif
    m_enableCheckBox->SetValue(pSolarSystemObj->Get_SolarSystemObjMode());
    m_BinningCtrl->Select(pCamera ? pCamera->GetBinning() - 1 : 0);
    m_saveVideoLogCheckBox->SetValue(pSolarSystemObj->GetVideoLogging());
    m_overrideCustomRate->SetValue(pFrame->m_planetToolCustomRatesEnabled);
    UpdateCustomRateLabels();
    SetEnabledState(this, pSolarSystemObj->Get_SolarSystemObjMode());

    // Set the initial state of the pause button
    m_PauseButton->SetLabel(pSolarSystemObj->GetDetectionPausedState() ? _("Resume") : _("Pause"));

    // Update mount states
    m_driveRate = (enum DriveRates) - 1;
    m_prevPointingSource = nullptr;
    m_prevMountConnected = false;
    m_prevMountCustomRateNonZero = false;
    m_updatingCustomRateFields = false;
    wxTimerEvent dummyEvent;
    OnPlanetaryTimer(dummyEvent);
    if (MinorBodyTrackingActive())
    {
        LockPosShiftParams const& shift = pFrame->pGuider->GetLockPosShiftParams();
        RefreshCustomRateShiftReadback(pPointingSource && pPointingSource->IsConnected() ?
                                           pPointingSource->GetDeclinationRadians() : UNKNOWN_DECLINATION,
                                       shift.shiftRate.X, shift.shiftRate.Y);
    }
    else if (m_overrideCustomRate->IsChecked() &&
             m_mountGuidingRate->GetSelection() != wxNOT_FOUND &&
             m_mountGuidingRate->GetStringSelection() == _("Custom") &&
             pPointingSource && pPointingSource->IsConnected())
    {
        enum DriveRates driveRate = driveSidereal;
        double raRate = 0.0;
        double decRate = 0.0;
        pPointingSource->GetTrackingRate(&driveRate, &raRate, &decRate, false);
        RefreshCustomRateReadback(pPointingSource->GetDeclinationRadians(), raRate, decRate);
    }
    UpdateCustomRateControlsEnabled();

    // Update camera settings
    m_DelayCtrl->SetValue(pFrame->GetTimeLapse());
    if (pCamera)
        m_GainCtrl->SetValue(pCamera->GetCameraGain());
    SyncCameraExposure(true);

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
    m_planetaryTimer.Start(1000);
}

PlanetToolWin::~PlanetToolWin(void)
{
    wxMutexLocker lock(pFrame->planetLock);
    m_planetaryTimer.Stop();
    pFrame->m_planetToolCustomRatesEnabled = m_overrideCustomRate->IsChecked();
    pFrame->pPlanetTool = nullptr;
}

void PlanetToolWin::OnEnableToggled(wxCommandEvent& event)
{
    bool enabled = m_enableCheckBox->IsChecked();
    GuiderMultiStar *pMultiGuider = dynamic_cast<GuiderMultiStar *>(pFrame->pGuider);

    if (enabled)
    {
        pSolarSystemObj->Set_SolarSystemObjMode(true);
        SetEnabledState(this, true);
    }
    else
    {
        pSolarSystemObj->Set_SolarSystemObjMode(false);
        SetEnabledState(this, false);
    }

    // Update elements display state
    pFrame->NotifyGuidingParam("Planet Mode", enabled);
    OnShowElementsClick(event);
}

void PlanetToolWin::OnSpinCtrl_minRadius(wxSpinDoubleEvent& event)
{
    int v = m_minRadius->GetValue();
    pSolarSystemObj->Set_minRadius(v < 1 ? 1 : v);
    pSolarSystemObj->RefreshMinMaxDiameters();
}

void PlanetToolWin::OnSpinCtrl_maxRadius(wxSpinDoubleEvent& event)
{
    int v = m_maxRadius->GetValue();
    pSolarSystemObj->Set_maxRadius(v < 1 ? 1 : v);
    pSolarSystemObj->RefreshMinMaxDiameters();
}

void PlanetToolWin::OnRoiModeClick(wxCommandEvent& event)
{
    bool enabled = m_RoiCheckBox->IsChecked();
    pSolarSystemObj->SetRoiEnableState(enabled);
    Debug.Write(wxString::Format("Solar/planetary: ROI %s\n", enabled ? "enabled" : "disabled"));
}

void PlanetToolWin::OnShowElementsClick(wxCommandEvent& event)
{
    bool enabled = m_ShowElements->IsChecked();
    pSolarSystemObj->SetShowFeaturesButtonState(enabled);
    if (pSolarSystemObj->Get_SolarSystemObjMode() && enabled)
        pSolarSystemObj->ShowVisualElements(true);
    else
        pSolarSystemObj->ShowVisualElements(false);
    pFrame->pGuider->Refresh();
}

#ifdef DEVELOPER_MODE
void PlanetToolWin::OnNoiseFilterClick(wxCommandEvent& event)
{
    bool enabled = m_NoiseFilter->IsChecked();
    pSolarSystemObj->SetNoiseFilterState(enabled);
    Debug.Write(wxString::Format("Solar/planetary: noise filter %s\n", enabled ? "enabled" : "disabled"));
}
#endif

void PlanetToolWin::OnSaveVideoLog(wxCommandEvent& event)
{
    bool enabled = m_saveVideoLogCheckBox->IsChecked();
    pSolarSystemObj->SetVideoLogging(enabled);
    Debug.Write(wxString::Format("Solar/planetary: video log %s\n", enabled ? "enabled" : "disabled"));
}

// Allow changing tracking state only when CTRL key is pressed
void PlanetToolWin::OnMountTrackingClick(wxCommandEvent& event)
{
    bool tracking = m_mountTrackigCheckBox->IsChecked();

    if (pPointingSource && pPointingSource->IsConnected())
    {
        if (wxGetKeyState(WXK_CONTROL))
        {
            pPointingSource->SetTracking(tracking);
        }
        pPointingSource->GetTracking(&tracking);
    }
    else
    {
        tracking = false;
    }

    pFrame->NotifyGuidingParam("Mount Tracking", tracking);
    m_mountTrackigCheckBox->SetValue(tracking);
}

void PlanetToolWin::OnOverrideCustomRateClick(wxCommandEvent& event)
{
    if (event.IsChecked())
    {
        int const customSel = m_mountGuidingRate->FindString(_("Custom"));
        if (customSel != wxNOT_FOUND)
            m_mountGuidingRate->SetSelection(customSel);
    }

    UpdateCustomRateControlsEnabled();
}

void PlanetToolWin::OnMinorBodyTrackingModeClick(wxCommandEvent& event)
{
    SetCustomRateFieldHighlight(false);
    UpdateCustomRateControlsEnabled();
}

void PlanetToolWin::OnHorizonsRateChanged(wxSpinDoubleEvent& event)
{
    if (!m_updatingCustomRateFields)
        SetCustomRateFieldHighlight(false);
}

void PlanetToolWin::OnCustomRateChar(wxKeyEvent& event)
{
    int const keyCode = event.GetKeyCode();
    if (keyCode < WXK_SPACE || keyCode == WXK_DELETE || keyCode == WXK_BACK || keyCode == WXK_TAB ||
        keyCode == WXK_LEFT || keyCode == WXK_RIGHT || keyCode == WXK_HOME || keyCode == WXK_END)
    {
        event.Skip();
        return;
    }

    if (IsCustomRateInputKey(keyCode))
    {
        event.Skip();
        return;
    }
}

void PlanetToolWin::OnApplyCustomRateClick(wxCommandEvent& event)
{
    ApplyCurrentCustomRate();
}

void PlanetToolWin::OnMinorBodyTrackingClick(wxCommandEvent& event)
{
    if (!pFrame->pGuider->IsGuiding() || !CanEditCustomRates())
        return;

    double raRate = 0.0;
    double decRate = 0.0;
    if (!GetCurrentCustomRateShiftRates(&raRate, &decRate))
    {
        Debug.Write("Solar/planetary: failed to convert custom rates for minor body tracking\n");
        return;
    }

    if (pPointingSource && pPointingSource->IsConnected())
    {
        pPointingSource->SetTrackingRate(driveSidereal);
        pPointingSource->SetTrackingRateOffsets(0.0, 0.0);
    }

    pFrame->pGuider->SetLockPosShiftRate(PHD_Point(raRate, decRate), UNIT_ARCSEC, true, false);
    pFrame->pGuider->EnableLockPosShift(true);
    m_prevMountCustomRateNonZero = false;
    Debug.Write(wxString::Format(
        "Solar/planetary: started minor body tracking via lock-position shift ra=%.6f dec=%.6f arcsec/hr\n",
        raRate, decRate));
    SetCustomRateFieldHighlight(true);
    UpdateCustomRateControlsEnabled();
}

void PlanetToolWin::OnStopMinorBodyTrackingClick(wxCommandEvent& event)
{
    if (!MinorBodyTrackingActive())
        return;

    pFrame->pGuider->EnableLockPosShift(false);
    Debug.Write("Solar/planetary: stopped minor body tracking via lock-position shift\n");
    SetCustomRateFieldHighlight(false);
    UpdateCustomRateControlsEnabled();
}

bool PlanetToolWin::CanApplyCustomMountRates() const
{
    return pPointingSource && pPointingSource->IsConnected() &&
           pPointingSource->CanSetRightAscensionRate() &&
           pPointingSource->CanSetDeclinationRate();
}

bool PlanetToolWin::CanEditCustomRates() const
{
    return m_overrideCustomRate->IsChecked() &&
           m_mountGuidingRate->GetSelection() != wxNOT_FOUND &&
           (m_mountGuidingRate->GetStringSelection() == _("Custom") || MinorBodyTrackingActive());
}

void PlanetToolWin::UpdateCustomRateRanges()
{
    m_Horizons_dRaCosDRate->SetRange(-MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR, MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR);
    m_Horizons_dDecRate->SetRange(-MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR, MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR);
    m_Horizons_dRaCosDRate->SetDigits(3);
    m_Horizons_dDecRate->SetDigits(3);
    m_updatingCustomRateFields = true;
    m_Horizons_dRaCosDRate->SetValue(wxClip(m_Horizons_dRaCosDRate->GetValue(), -MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR,
                                            MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR));
    m_Horizons_dDecRate->SetValue(wxClip(m_Horizons_dDecRate->GetValue(), -MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR,
                                         MAX_HORIZONS_TRACK_RATE_ARCSEC_PER_HOUR));
    m_updatingCustomRateFields = false;
}

void PlanetToolWin::UpdateCustomRateLabels()
{
    m_customRateRaLabel->SetLabel(_("dRA*cosD/dt") + _(":"));
    m_customRateDecLabel->SetLabel(_("d(DEC)") + _(":"));
    m_customRateRaLabel->SetToolTip(
        _("Horizons RA*cos(Dec) rate, arcsec/hour. Apply it only when the mount is synced to the target, or "
          "the mount RA rate will be wrong. Limit is 5x sidereal rate."));
    m_customRateDecLabel->SetToolTip(_("Horizons rate of change in Dec, arcsec/hour. Limit is 5x sidereal rate."));
    m_Horizons_dRaCosDRate->UnsetToolTip();
    m_Horizons_dDecRate->UnsetToolTip();
    UpdateCustomRateRanges();
    Layout();
}

void PlanetToolWin::UpdateCustomRateControlsEnabled()
{
    bool const editable = CanEditCustomRates();
    bool const shiftActive = MinorBodyTrackingActive();
    bool const customRatesVisible = m_overrideCustomRate->IsChecked();
    bool const canApplyCustomMountRates = CanApplyCustomMountRates();
    if ((shiftActive || !canApplyCustomMountRates) && !m_minorBodyTrackingMode->GetValue())
        m_minorBodyTrackingMode->SetValue(true);
    bool const minorBodyMode = m_minorBodyTrackingMode->GetValue() || shiftActive;
    bool const showApplyButton = customRatesVisible && canApplyCustomMountRates && !minorBodyMode;
    bool const showMinorBodyButtons = customRatesVisible && minorBodyMode;
    bool const visibilityChanged = m_mountGroup->IsShown(m_customRatesGroup) != customRatesVisible;
    bool const applyButtonVisibilityChanged = m_customRatesGroup->IsShown(m_applyCustomRateButton) != showApplyButton;
    bool const minorBodyButtonsVisibilityChanged =
        m_customRatesGroup->IsShown(m_minorBodyTrackingButton) != showMinorBodyButtons ||
        m_customRatesGroup->IsShown(m_stopMinorBodyTrackingButton) != showMinorBodyButtons;
    bool const layoutChanged = visibilityChanged || applyButtonVisibilityChanged ||
                               minorBodyButtonsVisibilityChanged;
    if (visibilityChanged)
        m_mountGroup->Show(m_customRatesGroup, customRatesVisible, true);
    m_customMountTrackingMode->Enable(customRatesVisible && canApplyCustomMountRates && !shiftActive);
    m_minorBodyTrackingMode->Enable(customRatesVisible);
    m_Horizons_dRaCosDRate->Enable(editable);
    m_Horizons_dDecRate->Enable(editable);
    m_customRatesGroup->Show(m_applyCustomRateButton, showApplyButton, true);
    m_customRatesGroup->Show(m_minorBodyTrackingButton, showMinorBodyButtons, true);
    m_customRatesGroup->Show(m_stopMinorBodyTrackingButton, showMinorBodyButtons, true);
    m_applyCustomRateButton->Enable(editable && !shiftActive && !minorBodyMode);
    m_minorBodyTrackingButton->Enable(editable && minorBodyMode && pFrame->pGuider->IsGuiding());
    m_minorBodyTrackingButton->SetLabel(shiftActive ? _("Update minor body rate") : _("Start minor body tracking"));
    m_stopMinorBodyTrackingButton->Enable(minorBodyMode && shiftActive);
    if (layoutChanged && GetSizer())
        GetSizer()->Layout();
    if (layoutChanged)
        Layout();
    if (layoutChanged && GetSizer())
    {
        SetSizerAndFit(GetSizer());
        SendSizeEvent(wxSEND_EVENT_POST);
    }
}

void PlanetToolWin::SetCustomRateFieldHighlight(bool applied)
{
    wxColour const color = applied ? wxColour(220, 255, 220) : wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW);
    m_Horizons_dRaCosDRate->SetBackgroundColour(color);
    m_Horizons_dDecRate->SetBackgroundColour(color);
    m_Horizons_dRaCosDRate->Refresh();
    m_Horizons_dDecRate->Refresh();
}

void PlanetToolWin::ApplyCurrentCustomRate()
{
    if (!CanEditCustomRates() || !pPointingSource || !pPointingSource->IsConnected())
        return;

    auto showFailure = [](const wxString& msg) {
        Debug.Write(wxString::Format("Solar/planetary: %s\n", msg));
        pFrame->Alert(msg, wxICON_WARNING);
    };

    auto unsupportedMessage = [](bool ra, bool dec) {
        if (ra && dec)
            return _("Setting custom tracking rate failed: driver doesn't support custom RA or Dec tracking rate.");
        if (ra)
            return _("Setting custom tracking rate failed: driver doesn't support custom RA tracking rate.");
        return _("Setting custom tracking rate failed: driver doesn't support custom Dec tracking rate.");
    };

    auto invalidRateMessage = []() {
        return _("Setting custom tracking rate failed: rate parameter is invalid.");
    };

    auto isInvalidRateError = [](const wxString& error) {
        wxString lower = error.Lower();
        return lower.Contains("invalid") || lower.Contains("not valid");
    };

    if (!CanApplyCustomMountRates())
    {
        SetCustomRateFieldHighlight(false);
        showFailure(unsupportedMessage(!pPointingSource->CanSetRightAscensionRate(),
                                       !pPointingSource->CanSetDeclinationRate()));
        UpdateCustomRateControlsEnabled();
        return;
    }

    double ra_offset = 0.0;
    double dec_offset = 0.0;
    if (!GetCurrentCustomRateMountOffsets(&ra_offset, &dec_offset))
    {
        SetCustomRateFieldHighlight(false);
        showFailure(invalidRateMessage());
        return;
    }

    Debug.Write(wxString::Format(
        "Solar/planetary: applying custom tracking rates (%s) ra=%.6f dec=%.6f -> mount ra=%.9f dec=%.9f\n",
        "Horizons", m_Horizons_dRaCosDRate->GetValue(), m_Horizons_dDecRate->GetValue(), ra_offset, dec_offset));

    const double rateTolerance = 1e-12;
    bool const needsRa = fabs(ra_offset) > rateTolerance;
    bool const needsDec = fabs(dec_offset) > rateTolerance;
    bool const missingRa = needsRa && !pPointingSource->CanSetRightAscensionRate();
    bool const missingDec = needsDec && !pPointingSource->CanSetDeclinationRate();
    if (missingRa || missingDec)
    {
        SetCustomRateFieldHighlight(false);
        showFailure(unsupportedMessage(missingRa, missingDec));
        return;
    }

    wxString setRateError;
    if (pPointingSource->SetTrackingRate(driveSidereal, &setRateError))
    {
        SetCustomRateFieldHighlight(false);
        showFailure(_("Setting custom tracking rate failed."));
        return;
    }

    wxString setOffsetsError;
    if (pPointingSource->SetTrackingRateOffsets(ra_offset, dec_offset, &setOffsetsError))
    {
        SetCustomRateFieldHighlight(false);
        if (isInvalidRateError(setOffsetsError))
            showFailure(invalidRateMessage());
        else
        {
            bool const unsupportedRa = needsRa && !pPointingSource->CanSetRightAscensionRate();
            bool const unsupportedDec = needsDec && !pPointingSource->CanSetDeclinationRate();
            if (unsupportedRa || unsupportedDec)
                showFailure(unsupportedMessage(unsupportedRa, unsupportedDec));
            else
                showFailure(_("Setting custom tracking rate failed."));
        }
        return;
    }

    m_driveRate = driveCustom;
    SetCustomRateFieldHighlight(true);
}

bool PlanetToolWin::GetCurrentCustomRateMountOffsets(double *raOffset, double *decOffset) const
{
    if (!raOffset || !decOffset || !pPointingSource || !pPointingSource->IsConnected())
        return false;

    return HorizonsRatesToMountOffsets(pPointingSource->GetDeclinationRadians(), m_Horizons_dRaCosDRate->GetValue(),
                                       m_Horizons_dDecRate->GetValue(), raOffset, decOffset);
}

bool PlanetToolWin::GetCurrentCustomRateShiftRates(double *raRate, double *decRate) const
{
    if (!raRate || !decRate || !pPointingSource || !pPointingSource->IsConnected())
        return false;

    return HorizonsRatesToShiftRates(pPointingSource->GetDeclinationRadians(), m_Horizons_dRaCosDRate->GetValue(),
                                     m_Horizons_dDecRate->GetValue(), raRate, decRate);
}

bool PlanetToolWin::MinorBodyTrackingActive() const
{
    LockPosShiftParams const& shift = pFrame->pGuider->GetLockPosShiftParams();
    return shift.shiftEnabled && shift.shiftUnits == UNIT_ARCSEC && shift.shiftIsMountCoords;
}

void PlanetToolWin::RefreshCustomRateShiftReadback(double declination, double raRate, double decRate)
{
    double customRaRate = 0.0;
    double customDecRate = 0.0;
    bool converted = pPointingSource && pPointingSource->IsConnected() &&
                     ShiftRatesToHorizonsRates(declination, raRate, decRate, &customRaRate, &customDecRate);

    if (converted)
    {
        m_updatingCustomRateFields = true;
        m_Horizons_dRaCosDRate->SetValue(customRaRate);
        m_Horizons_dDecRate->SetValue(customDecRate);
        m_updatingCustomRateFields = false;
    }
    else
    {
        m_updatingCustomRateFields = true;
        m_Horizons_dRaCosDRate->SetValue(0.0);
        m_Horizons_dDecRate->SetValue(0.0);
        m_updatingCustomRateFields = false;
    }
}

void PlanetToolWin::RefreshCustomRateReadback(double declination, double raRate, double decRate)
{
    double customRaRate = 0.0;
    double customDecRate = 0.0;
    bool converted = pPointingSource && pPointingSource->IsConnected() &&
                     MountOffsetsToHorizonsRates(declination, raRate, decRate, &customRaRate, &customDecRate);

    if (converted)
    {
        m_updatingCustomRateFields = true;
        m_Horizons_dRaCosDRate->SetValue(customRaRate);
        m_Horizons_dDecRate->SetValue(customDecRate);
        m_updatingCustomRateFields = false;
    }
    else
    {
        m_updatingCustomRateFields = true;
        m_Horizons_dRaCosDRate->SetValue(0.0);
        m_Horizons_dDecRate->SetValue(0.0);
        m_updatingCustomRateFields = false;
    }
}

// Called once in a while to update the UI controls
void PlanetToolWin::OnPlanetaryTimer(wxTimerEvent& event)
{
    enum DriveRates driveRate = driveSidereal;
    double raRate = 0, decRate = 0;
    double declination = UNKNOWN_DECLINATION;
    bool tracking = false;
    bool mountRateValid = false;
    bool need_update = false;
    bool const shiftActive = MinorBodyTrackingActive();

    // Update UI controls which can be changed via event server
    if (pSolarSystemObj->GetPlanetaryModeUpdate())
    {
        pSolarSystemObj->SetPlanetaryModeUpdate(false);
        m_enableCheckBox->SetValue(pSolarSystemObj->Get_SolarSystemObjMode());
    }

    if (pSolarSystemObj->GetDisableCustomRateOverride())
    {
        m_overrideCustomRate->SetValue(false);
        UpdateCustomRateControlsEnabled();
        Debug.Write("Solar/planetary: custom tracking override disabled by external tracking-rate update\n");
    }

    // Update pause button state to sync with guiding state
    bool paused = pSolarSystemObj->GetDetectionPausedState() && pFrame->pGuider->IsGuiding();
    pSolarSystemObj->SetDetectionPausedState(paused);
    m_PauseButton->SetLabel(paused ? _("Resume") : _("Pause"));
    SetEnabledState(this, pSolarSystemObj->Get_SolarSystemObjMode());
    if (!paused && pauseAlert)
    {
        pauseAlert = false;
        pFrame->ClearAlert(planetaryPauseAlertMsg);
    }

    if (pPointingSource && pPointingSource->IsConnected())
    {
        // Currently not supporting INDI mounts
        if (pPointingSource->Name().StartsWith(_("INDI Mount")))
        {
            m_mountTrackigCheckBox->Enable(false);
            m_mountGuidingRate->Enable(false);
            return;
        }
        pPointingSource->GetTracking(&tracking);
        mountRateValid = !pPointingSource->GetTrackingRate(&driveRate, &raRate, &decRate, false);
        declination = pPointingSource->GetDeclinationRadians();
        m_mountTrackigCheckBox->Enable(true);
        m_mountGuidingRate->Enable(tracking && !shiftActive);
    }
    else
    {
        m_mountTrackigCheckBox->Enable(false);
        m_mountGuidingRate->Enable(false);
    }
    m_mountTrackigCheckBox->SetValue(tracking);

    // Look for changes in the mount connection state
    if (m_prevPointingSource != pPointingSource ||
        (m_prevMountConnected != (pPointingSource && pPointingSource->IsConnected())))
    {
        m_prevMountCustomRateNonZero = false;
        m_mountGuidingRate->Clear();
        if (pPointingSource && pPointingSource->IsConnected())
        {
            for (int i = 0; i < driveMaxRate; i++)
            {
                enum DriveRates rate = (enum DriveRates) i;
                m_mountGuidingRate->Append(pPointingSource->m_mountRates[i].name);
            }
        }
        need_update = true;
    }
    m_prevPointingSource = pPointingSource;
    m_prevMountConnected = pPointingSource && pPointingSource->IsConnected();

    // Iterate through the available rates in the m_mountGuidingRate combo box and select the current rate
    int new_selection = -1;
    wxString rateStr = wxEmptyString;
    bool mountCustomRateNonZero = false;
    for (int i = 0; mountRateValid && i < m_mountGuidingRate->GetCount(); i++)
    {
        rateStr = m_mountGuidingRate->GetString(i);
        const double tolerance = 0.00001;
        if ((rateStr == _("Sidereal") && driveRate == driveSidereal) ||
            (rateStr == _("Lunar") && driveRate == driveLunar) ||
            (rateStr == _("Solar") && driveRate == driveSolar) ||
            (rateStr == _("King")  && driveRate == driveKing))
        {
            // Special handling of custom RA/DEC offsets from SideReal rate
            if (driveRate == driveSidereal)
            {
                // Compensate for possible reversal in South hemisphere
                if (raRate > 15.041067)
                    raRate -= 15.041067 * 2;

                // Check for lunar rate offset
                if ((fabs(raRate - RA_LUNAR_RATE_OFFSET) < tolerance) && (fabs(decRate) < tolerance))
                {
                    rateStr = _("Lunar");
                    new_selection = driveRate = driveLunar;
                    break;
                }
                // Check for solar rate offset
                else if ((fabs(raRate - RA_SOLAR_RATE_OFFSET) < tolerance) && (fabs(decRate) < tolerance))
                {
                    rateStr = _("Solar");
                    new_selection = driveRate = driveSolar;
                    break;
                }
                else if ((fabs(raRate) > tolerance) || (fabs(decRate) > tolerance))
                {
                    rateStr = _("Custom");
                    mountCustomRateNonZero = true;
                    new_selection = driveRate = driveCustom; // custom rate
                    break;
                }
                else if (shiftActive)
                {
                    rateStr = _("Custom");
                    new_selection = driveRate = driveCustom;
                    break;
                }
                else if (m_mountGuidingRate->GetSelection() != wxNOT_FOUND &&
                         m_mountGuidingRate->GetStringSelection() == _("Custom") &&
                         !m_prevMountCustomRateNonZero)
                {
                    rateStr = _("Custom");
                    new_selection = driveRate = driveCustom;
                    break;
                }
            }
            new_selection = i;
            break;
        }
    }
    need_update |= (new_selection != m_driveRate);

    if (((m_driveRate != driveRate) || need_update) && new_selection != -1)
    {
        Debug.Write(wxString::Format("solar/planetary: mount tracking rate = %s\n", rateStr));
        m_mountGuidingRate->SetSelection(new_selection);
    }
    if (driveRate != driveCustom &&
        (driveRate == driveLunar || driveRate == driveSolar || driveRate == driveKing ||
         (driveRate == driveSidereal && m_prevMountCustomRateNonZero && !mountCustomRateNonZero)))
    {
        m_updatingCustomRateFields = true;
        m_Horizons_dRaCosDRate->SetValue(0.0);
        m_Horizons_dDecRate->SetValue(0.0);
        m_updatingCustomRateFields = false;
    }
    m_driveRate = driveRate;
    m_prevMountCustomRateNonZero = mountCustomRateNonZero;
    UpdateCustomRateControlsEnabled();

    if (!m_overrideCustomRate->IsChecked())
    {
        if (MinorBodyTrackingActive())
        {
            LockPosShiftParams const& shift = pFrame->pGuider->GetLockPosShiftParams();
            RefreshCustomRateShiftReadback(declination, shift.shiftRate.X, shift.shiftRate.Y);
        }
        else
            RefreshCustomRateReadback(declination, raRate, decRate);
    }

    // Update camera binning
    if (pCamera)
    {
        int localBinning = m_BinningCtrl->GetSelection();
        if (pCamera->GetBinning() != localBinning + 1)
        {
            m_BinningCtrl->Select(pCamera->GetBinning() - 1);
        }
    }

    // Update min/max radius updated by the PHD2 client - in disk detection mode
    if (pSolarSystemObj->GetMinMaxDiametersUpdate() && !pSolarSystemObj->GetSurfaceTrackingState())
    {
        m_minRadius->SetValue(pSolarSystemObj->Get_minRadius());
        m_maxRadius->SetValue(pSolarSystemObj->Get_maxRadius());
    }

    // Check 500 msec rule
    CheckMinExposureDuration();
}

void PlanetToolWin::OnMountTrackingRateClick(wxCommandEvent& event)
{
    SetCustomRateFieldHighlight(false);
    enum DriveRates driveRate = driveSidereal;
    if (pPointingSource && pPointingSource->IsConnected())
    {
        wxString rateStr = "Sidereal";
        double ra_offset = 0.0;
        double dec_offset = 0.0;
        int sel = m_mountGuidingRate->GetSelection();
        if (sel != wxNOT_FOUND)
        {
            rateStr = m_mountGuidingRate->GetString(sel);
            if (rateStr == _("Custom") && !m_overrideCustomRate->IsChecked())
                m_overrideCustomRate->SetValue(true);
            if (rateStr != _("Custom") && m_overrideCustomRate->IsChecked())
                m_overrideCustomRate->SetValue(false);
            if (rateStr == _("Sidereal"))
                driveRate = driveSidereal;
            else if (rateStr == _("Lunar"))
            {
                driveRate = driveLunar;
                ra_offset = RA_LUNAR_RATE_OFFSET;
            }
            else if (rateStr == _("Solar"))
            {
                driveRate = driveSolar;
                ra_offset = RA_SOLAR_RATE_OFFSET;
            }
            else if (rateStr == _("King"))
                driveRate = driveKing;
            else
            {
                driveRate = driveCustom;
            }
        }

        UpdateCustomRateControlsEnabled();

        Debug.Write(wxString::Format("Solar/planetary: setting mount tracking rate to %s\n", rateStr));
        if (pPointingSource->m_mountRates[driveRate].canSet)
        {
            pPointingSource->SetTrackingRate(driveRate == driveCustom ? driveSidereal : driveRate);
            if (!(driveRate == driveCustom && m_overrideCustomRate->IsChecked() && pSolarSystemObj->Get_SolarSystemObjMode()))
                pPointingSource->SetTrackingRateOffsets(0, 0);

            m_driveRate = driveRate;
        }
        else
        {
            m_mountGuidingRate->SetSelection((int) driveRate);
        }

        // Set custom rate offsets for EQMOD mounts
        if (pPointingSource->Name().StartsWith(_("EQMOD ASCOM")) &&
            !(driveRate == driveCustom && m_overrideCustomRate->IsChecked() && pSolarSystemObj->Get_SolarSystemObjMode()))
        {
            Debug.Write(wxString::Format("Solar/planetary: setting RA tracking offset %.6f for EQMOD ASCOM\n", ra_offset));
            pPointingSource->SetTrackingRateOffsets(ra_offset, 0);
        }
    }
}

void PlanetToolWin::OnTrackingRateMouseWheel(wxMouseEvent& event)
{
    // Do nothing here - we don't want to change the tracking rate with the mouse wheel
}

void PlanetToolWin::OnExposureChanged(wxSpinDoubleEvent& event)
{
    int expMsec = m_ExposureCtrl->GetValue();
    expMsec = wxMin(expMsec, PT_CAMERA_EXPOSURE_MAX);
    expMsec = wxMax(expMsec, PT_CAMERA_EXPOSURE_MIN);
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

void PlanetToolWin::OnBinningSelected(wxCommandEvent& event)
{
    int sel = m_BinningCtrl->GetSelection();
    AdvancedDialog *pAdvancedDlg = pFrame->pAdvancedDialog;
    if (pAdvancedDlg)
    {
        pAdvancedDlg->SetBinning(sel + 1);
        if (pCamera && pCamera->Connected && (pCamera->GetBinning() != sel + 1))
            pAdvancedDlg->MakeImageScaleAdjustments();
    }
    if (pCamera)
    {
        pCamera->SetBinning(sel + 1);
    }
}

void PlanetToolWin::UpdateStatus()
{
    bool enabled = pSolarSystemObj->Get_SolarSystemObjMode();
    bool surfaceTracking = pSolarSystemObj->GetSurfaceTrackingState();
    bool enableLocalControls = true;

    // Update solar/planetary mode detection controls
    m_minRadius->Enable(enabled && !surfaceTracking);
    m_maxRadius->Enable(enabled && !surfaceTracking);

#if defined(FRAME_MONITOR_CAMERA)
    if (pCamera && pCamera->Name == FRAME_MONITOR_CAMERA)
    {
        m_RoiCheckBox->SetValue(false);
        m_ShowElements->SetValue(false);
        pSolarSystemObj->SetRoiEnableState(false);
        pSolarSystemObj->ShowVisualElements(false);
        enableLocalControls = false;
    }
#endif

    m_enableCheckBox->Show(enableLocalControls);
    m_planetPanel->Show(enableLocalControls);
    m_RoiCheckBox->Enable(enabled && !surfaceTracking);
    m_ShowElements->Enable(enabled && !surfaceTracking);
#ifdef DEVELOPER_MODE
    m_NoiseFilter->Enable(enabled);
#endif
    m_saveVideoLogCheckBox->Enable(enabled);
    m_overrideCustomRate->Enable(true);

    // Update slider states
    m_thresholdSlider->Enable(enabled && enableLocalControls && !surfaceTracking);

    // Update checkmark state in tools menu
    pFrame->m_PlanetaryMenuItem->Check(enabled);

    // Toggle the visibility of solar/planetary stats grid
    pFrame->pStatsWin->ShowPlanetStats(enabled);

    // Pause solar system object guiding can be enabled only when guiding is still active
    m_PauseButton->Enable(enabled && pFrame->pGuider->IsGuiding());

    // Update dialog layout
    if (m_panelVisible != enableLocalControls)
    {
        Layout();
        SetSizerAndFit(GetSizer());
        m_panelVisible = enableLocalControls;
    }
}

void PlanetToolWin::OnKeyDown(wxKeyEvent& event)
{
    if (event.AltDown() && m_MouseHoverFlag)
    {
        m_CloseButton->SetLabel(_("Reset"));
    }
    event.Skip(); // Ensure that other key handlers are not skipped
}

void PlanetToolWin::OnKeyUp(wxKeyEvent& event)
{
    m_CloseButton->SetLabel(_("Ok"));
    event.Skip();
}

void PlanetToolWin::OnMouseEnterCloseBtn(wxMouseEvent& event)
{
    m_MouseHoverFlag = true;
    if (wxGetKeyState(WXK_ALT))
    {
        m_CloseButton->SetLabel(_("Reset"));
    }
    event.Skip();
}

void PlanetToolWin::OnMouseLeaveCloseBtn(wxMouseEvent& event)
{
    m_MouseHoverFlag = false;
    m_CloseButton->SetLabel(_("Ok"));
    event.Skip();
}

void PlanetToolWin::OnThresholdChanged(wxCommandEvent& event)
{
    int highThreshold = event.GetInt();
    highThreshold = wxMin(highThreshold, PT_HIGH_THRESHOLD_MAX);
    highThreshold = wxMax(highThreshold, PT_THRESHOLD_MIN);
    int lowThreshold = wxMax(highThreshold / 2, PT_THRESHOLD_MIN);
    pSolarSystemObj->Set_lowThreshold(lowThreshold);
    pSolarSystemObj->Set_highThreshold(highThreshold);
    pSolarSystemObj->RestartSimulatorErrorDetection();
}

static void SuppressPausePlanetDetection(intptr_t)
{
    pConfig->Global.SetBoolean(PausePlanetDetectionAlertEnabledKey(), false);
}

void PlanetToolWin::OnPauseButton(wxCommandEvent& event)
{
    // Toggle solar system object detection pause state depending if guiding is actually active
    bool paused = !pSolarSystemObj->GetDetectionPausedState() && pFrame->pGuider->IsGuiding();
    pSolarSystemObj->SetDetectionPausedState(paused);
    m_PauseButton->SetLabel(paused ? _("Resume") : _("Pause"));
    SetEnabledState(this, pSolarSystemObj->Get_SolarSystemObjMode());

    // Display special message if detection is paused
    if (paused)
    {
        pauseAlert = true;
        pFrame->SuppressibleAlert(PausePlanetDetectionAlertEnabledKey(), planetaryPauseAlertMsg, SuppressPausePlanetDetection,
                                  0);
    }
    else if (pauseAlert)
    {
        pauseAlert = false;
        pFrame->ClearAlert(planetaryPauseAlertMsg);
    }
}

void PlanetToolWin::OnClose(wxCloseEvent& evt)
{
    pFrame->m_PlanetaryMenuItem->Check(pSolarSystemObj->Get_SolarSystemObjMode());
    pSolarSystemObj->SetShowFeaturesButtonState(false);
    pSolarSystemObj->ShowVisualElements(false);
    pFrame->pGuider->Refresh();

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
        pSolarSystemObj->Set_minRadius(PT_MIN_RADIUS_DEFAULT);
        pSolarSystemObj->Set_maxRadius(PT_MAX_RADIUS_DEFAULT);
        pSolarSystemObj->Set_lowThreshold(PT_HIGH_THRESHOLD_DEFAULT / 2);
        pSolarSystemObj->Set_highThreshold(PT_HIGH_THRESHOLD_DEFAULT);
#ifdef DEVELOPER_MODE
        pSolarSystemObj->SetNoiseFilterState(false);
#endif

        m_minRadius->SetValue(pSolarSystemObj->Get_minRadius());
        m_maxRadius->SetValue(pSolarSystemObj->Get_maxRadius());
        m_thresholdSlider->SetValue(pSolarSystemObj->Get_highThreshold());
#ifdef DEVELOPER_MODE
        m_NoiseFilter->SetValue(pSolarSystemObj->GetNoiseFilterState());
#endif
    }
    else
        this->Close();
}

void PlanetToolWin::CheckMinExposureDuration()
{
    wxString alertMsg = _(
        "Warning: the sum of camera exposure and time lapse duration must be at least 500 msec (recommended 2000-3000 msec)!");
    int delayMsec = m_DelayCtrl->GetValue();
    int exposureMsec = m_ExposureCtrl->GetValue();
    if (delayMsec + exposureMsec < 500)
        pFrame->Alert(alertMsg, wxICON_WARNING);
    else
        pFrame->ClearAlert(alertMsg);
}

void PlanetToolWin::SyncCameraExposure(bool init)
{
    int exposureMsec;
    bool auto_exp;
    if (!pFrame->GetExposureInfo(&exposureMsec, &auto_exp))
    {
        exposureMsec = wxMax(exposureMsec, PT_CAMERA_EXPOSURE_MIN);
        exposureMsec = wxMin(exposureMsec, PT_CAMERA_EXPOSURE_MAX);
        pFrame->SetExposureDuration(exposureMsec, true);
    }
    else
    {
        exposureMsec = pConfig->Profile.GetInt("/ExposureDurationMs", 1000);
    }
    if (init || exposureMsec != m_ExposureCtrl->GetValue())
    {
        m_ExposureCtrl->SetValue(exposureMsec);
        if (exposureMsec != m_ExposureCtrl->GetValue())
        {
            exposureMsec = m_ExposureCtrl->GetValue();
            pFrame->SetExposureDuration(exposureMsec, true);
        }
    }
    CheckMinExposureDuration();
}

// Sync local camera settings with the main frame changes
void PlanetToolWin::OnAppStateNotify(wxCommandEvent& event)
{
    SyncCameraExposure();

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

wxWindow *PlanetTool::CreatePlanetToolWindow()
{
    return new PlanetToolWin();
}
