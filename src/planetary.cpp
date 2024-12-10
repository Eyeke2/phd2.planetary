/*
 *  planetary.cpp
 *  PHD Guiding
 *
 *  Solar, lunar and planetary detection extensions by Leo Shatz
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

#include "phd.h"
#include "planetary.h"
#include "planetary_tool.h"

#include <algorithm>
#include <numeric>

#if ((wxMAJOR_VERSION < 3) && (wxMINOR_VERSION < 9))
# define wxPENSTYLE_DOT wxDOT
#endif

// Using OpenCV namespace
using namespace cv;

// Gaussian weights lookup table
#define GAUSSIAN_SIZE 2000
static float gaussianWeight[GAUSSIAN_SIZE];

// Homography distance threshold
#define HOMOGRAPHY_DIST_THRESHOLD 5.0

// Initialize solar/planetary detection module
SolarSystemObject::SolarSystemObject()
{
    m_requestPlanetaryModeUpdate = false;
    m_paramEnabled = false;
    m_paramDetectionPaused = false;
    m_paramSurfaceTracking = false;
    m_paramRoiEnabled = false;
    m_paramShowElementsButtonState = false;

    m_prevCaptureActive = false;
    m_detected = false;
    m_radius = 0;
    m_searchRegion = 0;
    m_prevSearchRegion = 0;
    m_starProfileSize = 50;
    m_measuringSharpnessMode = false;
    m_unknownHFD = true;
    m_focusSharpness = 0;
    m_paramNoiseFilterState = false;
    m_requestSurfModeUpdate = false;

    int cliplimit = 2;
    m_surf.clahe = createCLAHE(cliplimit);
    m_surfaceDetectionParamsChanging = false;
    m_forceReferenceFrameSwitch = false;
    m_surf.referenceKeypoints.clear();
    m_surf.inlierPoints.clear();
    m_surf.guidingFixationPointValid = false;
    m_surf.surfaceFixationPoint = Point2f(0, 0);
    m_surf.guidingFixationPoint = Point2f(0, 0);
    m_surf.trackingQuality = 0;

    m_cameraSimulationMove = Point2f(0, 0);
    m_cameraSimulationRefPoint = Point2f(0, 0);
    m_cameraSimulationRefPointValid = false;
    m_simulationZeroOffset = false;
    m_center_x = m_center_y = 0;
    m_origPoint = Point2f(0, 0);

    m_cachedScaledWidth = 0;
    m_cachedScaledHeight = 0;
    m_cachedTrackerImage = NULL;

    m_userLClick = false;
    m_roiActive = false;
    m_detectionCounter = 0;
    m_clicked_x = 0;
    m_clicked_y = 0;
    m_prevClickedPoint = Point2f(0, 0);
    m_diskContour.clear();
    m_showVisualElements = false;
    m_showMinMaxDiameters = false;
    m_frameWidth = 0;
    m_frameHeight = 0;
    m_eccentricity = 0;
    m_angle = 0;

    m_prevMountTracking = false;
    m_prevTrackingRate = wxEmptyString;
    m_videoLogEnabled = false;
    m_SER = nullptr;

    m_StarFindMode_Saved = Star::FIND_CENTROID;

    // Build gaussian weighting function table used for circle feature detection
    float sigma = 1.0;
    memset(gaussianWeight, 0, sizeof(gaussianWeight));
    for (double x = 0; x < 20; x += 0.01)
    {
        int i = x * 100 + 0.5;
        if (i < GAUSSIAN_SIZE)
            gaussianWeight[i] += exp(-(pow(x, 2) / (2 * pow(sigma, 2))));
    }

    // Load lock target bitmap
#include "icons/target_lock_ok.png.h"
    wxBitmap lockTargetBitmapOk(wxBITMAP_PNG_FROM_DATA(target_lock_ok));
    m_lockTargetImageOk = wxImage(lockTargetBitmapOk.ConvertToImage());

    // Extract width and height from IHDR chunk data (9-12 bytes for width, 13-16 bytes for height)
    const uchar *ihdrChunk1 = target_lock_ok_png + 16;
    m_lockTargetWidthOk = (ihdrChunk1[0] << 24) | (ihdrChunk1[1] << 16) | (ihdrChunk1[2] << 8) | ihdrChunk1[3];
    m_lockTargetHeightOk = (ihdrChunk1[4] << 24) | (ihdrChunk1[5] << 16) | (ihdrChunk1[6] << 8) | ihdrChunk1[7];

#include "icons/target_lock_bad.png.h"
    wxBitmap lockTargetBitmapBad(wxBITMAP_PNG_FROM_DATA(target_lock_bad));
    m_lockTargetImageBad = wxImage(lockTargetBitmapBad.ConvertToImage());

    // Extract width and height from IHDR chunk data (9-12 bytes for width, 13-16 bytes for height)
    const uchar *ihdrChunk2 = target_lock_ok_png + 16;
    m_lockTargetWidthBad = (ihdrChunk2[0] << 24) | (ihdrChunk2[1] << 16) | (ihdrChunk2[2] << 8) | ihdrChunk2[3];
    m_lockTargetHeightBad = (ihdrChunk2[4] << 24) | (ihdrChunk2[5] << 16) | (ihdrChunk2[6] << 8) | ihdrChunk2[7];

    // Get initial values of the solar system object detection state and parameters from configuration
    SetSurfaceTrackingState(pConfig->Profile.GetBoolean("/PlanetTool/surface_tracking", false));
#ifdef DEVELOPER_MODE
    SetNoiseFilterState(pConfig->Profile.GetBoolean("/PlanetTool/noise_filter", false));
#endif

    // Enforce valid range limits on solar system object detection parameters while restoring from configuration
    m_paramMinRadius = pConfig->Profile.GetInt("/PlanetTool/min_radius", PT_MIN_RADIUS_DEFAULT);
    m_paramMinRadius = wxMax(PT_RADIUS_MIN, wxMin(PT_RADIUS_MAX, m_paramMinRadius));
    m_paramMaxRadius = pConfig->Profile.GetInt("/PlanetTool/max_radius", PT_MAX_RADIUS_DEFAULT);
    m_paramMaxRadius = wxMax(PT_RADIUS_MIN, wxMin(PT_RADIUS_MAX, m_paramMaxRadius));
    m_paramLowThreshold = pConfig->Profile.GetInt("/PlanetTool/high_threshold", PT_HIGH_THRESHOLD_DEFAULT) / 2;
    m_paramLowThreshold = wxMax(PT_THRESHOLD_MIN, wxMin(PT_LOW_THRESHOLD_MAX, m_paramLowThreshold));
    m_paramHighThreshold = pConfig->Profile.GetInt("/PlanetTool/high_threshold", PT_HIGH_THRESHOLD_DEFAULT);
    m_paramHighThreshold = wxMax(PT_THRESHOLD_MIN, wxMin(PT_HIGH_THRESHOLD_MAX, m_paramHighThreshold));
    m_paramMinHessian = pConfig->Profile.GetInt("/PlanetTool/min_hessian", PT_MIN_HESSIAN_UI_DEFAULT);
    m_paramMinHessian = wxMax(PT_MIN_HESSIAN_MIN, wxMin(PT_MIN_HESSIAN_MAX, m_paramMinHessian));
    m_paramMaxFeatures = pConfig->Profile.GetInt("/PlanetTool/max_features", PT_MAX_SURFACE_FEATURES);
    m_paramMaxFeatures = wxMax(PT_MIN_SURFACE_FEATURES, wxMin(PT_MAX_SURFACE_FEATURES, m_paramMaxFeatures));

    // Save PHD2 settings we change for solar system object guiding
    m_phd2_MassChangeThresholdEnabled = pConfig->Profile.GetBoolean("/guider/onestar/MassChangeThresholdEnabled", false);
    m_phd2_UseSubframes = pConfig->Profile.GetBoolean("/camera/UseSubframes", false);
    m_phd2_MultistarEnabled = pConfig->Profile.GetBoolean("/guider/multistar/enabled", true);

    // Remove the alert dialog setting for pausing solar/planetary detection
    pConfig->Global.DeleteEntry(PausePlanetDetectionAlertEnabledKey());

    // Initialize non-free OpenCV components
    // Note: this may cause a small and limited memory leak.
    bool nonfreeInit = initModule_nonfree();
    Debug.Write(wxString::Format("OpenCV nonfree module initialization status: %d\n", nonfreeInit));
    assert(nonfreeInit);
}

SolarSystemObject::~SolarSystemObject()
{
    delete m_SER;

    // Save all detection parameters
    pConfig->Profile.SetBoolean("/PlanetTool/surface_tracking", GetSurfaceTrackingState());
#ifdef DEVELOPER_MODE
    pConfig->Profile.SetBoolean("/PlanetTool/noise_filter", GetNoiseFilterState());
#endif
    pConfig->Profile.SetInt("/PlanetTool/min_radius", Get_minRadius());
    pConfig->Profile.SetInt("/PlanetTool/max_radius", Get_maxRadius());
    pConfig->Profile.SetInt("/PlanetTool/high_threshold", Get_highThreshold());
    pConfig->Profile.SetInt("/PlanetTool/min_hessian", Get_minHessian());
    pConfig->Profile.SetInt("/PlanetTool/max_features", Get_maxFeatures());
    pConfig->Flush();
}

// Is planetary/solar detection enabled
bool SolarSystemObject::Get_SolarSystemObjMode()
{
    return m_paramEnabled;
}

// Set/reset planetary/solar detection mode
void SolarSystemObject::Set_SolarSystemObjMode(bool enabled)
{
    // Don't change the state if it's already set
    if (m_paramEnabled == enabled)
        return;

    // Enable/disable planetary/solar detection mode
    m_paramEnabled = enabled;

    if (wxThread::IsMain())
    {
        GuiderMultiStar *pMultiGuider = dynamic_cast<GuiderMultiStar *>(pFrame->pGuider);

        if (enabled)
        {
            if (pFrame->GetStarFindMode() != Star::FIND_PLANET)
            {
                m_StarFindMode_Saved = pFrame->GetStarFindMode();
                pFrame->SetStarFindMode(Star::FIND_PLANET);
            }
            pFrame->m_PlanetaryMenuItem->Check(true);

            // Disable mass change threshold
            if (pMultiGuider)
            {
                m_phd2_MassChangeThresholdEnabled = pMultiGuider->GetMassChangeThresholdEnabled();
                pMultiGuider->SetMassChangeThresholdEnabled(false);
                pConfig->Profile.SetBoolean("/guider/onestar/MassChangeThresholdEnabled", m_phd2_MassChangeThresholdEnabled);
            }

            // Make sure lock position shift is disabled
            pFrame->pGuider->EnableLockPosShift(false);

            // Disable subframes
            if (pCamera)
            {
                pConfig->Profile.SetBoolean("/camera/UseSubframes", pCamera->UseSubframes);
                m_phd2_UseSubframes = pCamera->UseSubframes;
                pCamera->UseSubframes = false;
            }

            // Disable multi-star mode
            m_phd2_MultistarEnabled = pFrame->pGuider->GetMultiStarMode();
            pFrame->pGuider->SetMultiStarMode(false);
            pConfig->Profile.SetBoolean("/guider/multistar/enabled", m_phd2_MultistarEnabled);

            Debug.Write(_("Solar/planetary guiding mode: enabled\n"));
        }
        else
        {
            pFrame->SetStarFindMode(m_StarFindMode_Saved);
            pFrame->m_PlanetaryMenuItem->Check(false);

            // Restore the previous state of the mass change threshold, subframes and multi-star mode
            if (pMultiGuider)
                pMultiGuider->SetMassChangeThresholdEnabled(m_phd2_MassChangeThresholdEnabled);
            if (pCamera)
                pCamera->UseSubframes = m_phd2_UseSubframes;
            pFrame->pGuider->SetMultiStarMode(m_phd2_MultistarEnabled);

            Debug.Write(_("Solar/planetary guiding mode: disabled\n"));
        }

        // Clear planetary/solar stats window
        pFrame->pStatsWin->ClearPlanetStats();
        pFrame->pStatsWin->ShowPlanetStats(enabled);

        // Request to update controls in UI dialog (if opened)
        SetPlanetaryModeUpdate(true);
    }
    else
    {
        SolarPlanetaryMessage *msg = new SolarPlanetaryMessage();
        msg->type = SolarPlanetaryMessage::PLANETARY_MODE_CHANGE;
        msg->enabled = enabled;
        wxThreadEvent *event = new wxThreadEvent(wxEVT_THREAD, SOLAR_PLANETARY_EVENT);
        event->SetExtraLong((long) msg);
        wxQueueEvent(pFrame, event);
    }
}

// Set surface detection parameters passed by the client application
void SolarSystemObject::Set_SurfaceDetectionParams(int minHessian, int maxFeatures)
{
    if (wxThread::IsMain())
    {
        if (minHessian >= 0)
            Set_minHessian(minHessian);
        if (maxFeatures >= 0)
            Set_maxFeatures(maxFeatures);
        m_forceReferenceFrameSwitch = true;

        // Request to update controls in UI dialog if opened
        SetPlanetaryModeUpdate(true);
    }
    else
    {
        SolarPlanetaryMessage *msg = new SolarPlanetaryMessage();
        msg->type = SolarPlanetaryMessage::SURFACE_PARAMS_CHANGE;
        msg->minHessian = minHessian;
        msg->maxFeatures = maxFeatures;
        wxThreadEvent *event = new wxThreadEvent(wxEVT_THREAD, SOLAR_PLANETARY_EVENT);
        event->SetExtraLong((long) msg);
        wxQueueEvent(pFrame, event);
    }
}

// Enable/disable surface tracking mode
void SolarSystemObject::Set_SurfaceDetectionMode(bool enabled)
{
    // Don't change the state if it's already set
    if (enabled == m_paramSurfaceTracking)
        return;

    // Request to set the new state
    SetSurfaceTrackingState(enabled);
    SetSurfaceModeUpdate(true);
}

// Report detected object size or sharpness depending on measurement mode
double SolarSystemObject::GetHFD()
{
    if (m_unknownHFD)
        return std::nan("1");
    if (m_measuringSharpnessMode)
        return m_focusSharpness;
    else
        return m_detected ? m_radius : 0;
}

wxString SolarSystemObject::GetHfdLabel()
{
    if (m_measuringSharpnessMode)
        return _("SHARPNESS: ");
    else
        return _("RADIUS: ");
}

bool SolarSystemObject::IsPixelMetrics()
{
    return Get_SolarSystemObjMode() ? !m_measuringSharpnessMode : true;
}

// Handle mouse wheel rotation event from Star Profile windows.
// Positive or negative indicates direction of rotation.
void SolarSystemObject::ZoomStarProfile(int rotation)
{
    // Reset profile zoom when user does L-click and hold Alt button
    if (rotation == 0)
    {
        if (wxGetKeyState(WXK_ALT))
            m_starProfileSize = 50;
    }
    else
    {
        const int maxStarProfileSize = wxMin(m_frameWidth, m_frameHeight) / 4;
        const int minStarProfileSize = 15;
        int starProfileSize = m_starProfileSize + ((rotation > 0) ? 5 : -5);
        starProfileSize = wxMin(starProfileSize, maxStarProfileSize);
        starProfileSize = wxMax(starProfileSize, minStarProfileSize);
        m_starProfileSize = starProfileSize;
    }
}

// Toggle between sharpness and radius display
void SolarSystemObject::ToggleSharpness()
{
    // In surface tracking mode sharpness is always displayed
    if (GetPlanetDetectMode() != DETECTION_MODE_SURFACE)
    {
        m_measuringSharpnessMode = !m_measuringSharpnessMode;
        m_unknownHFD = true;
    }
}

// The Sobel operator can be used to detect edges in an image, which are more pronounced in
// focused images. You can apply the Sobel operator to the image and calculate the sum or mean
// of the absolute values of the gradients.
double SolarSystemObject::ComputeSobelSharpness(const Mat& img)
{
    Mat grad_x, grad_y;
    Sobel(img, grad_x, CV_32F, 1, 0);
    Sobel(img, grad_y, CV_32F, 0, 1);

    Mat grad;
    magnitude(grad_x, grad_y, grad);

    double sharpness = cv::mean(grad)[0];
    return sharpness;
}

// Calculate focus metrics around the updated tracked position
double SolarSystemObject::CalcSharpness(Mat& FullFrame, Point2f& clickedPoint, bool detectionResult)
{
    double scaleFactor;
    cv::Scalar meanSignal;
    Mat focusRoi;
    int focusX;
    int focusY;

    if (detectionResult)
    {
        focusX = m_center_x;
        focusY = m_center_y;
    }
    else if (norm(clickedPoint))
    {
        focusX = clickedPoint.x;
        focusY = clickedPoint.y;
    }
    else
    {
        // Compute scaling factor to normalize the signal
        meanSignal = cv::mean(focusRoi);
        scaleFactor = meanSignal[0] ? (65536.0 / 256) / meanSignal[0] : 1.0;

        // For failed auto selected star use entire frame for sharpness calculation
        FullFrame.convertTo(focusRoi, CV_32F, scaleFactor);
        return ComputeSobelSharpness(focusRoi);
    }

    const int focusSize = (GetPlanetDetectMode() == DETECTION_MODE_SURFACE) ? 200 : m_paramMaxRadius * 3 / 2.0;
    focusX = wxMax(0, focusX - focusSize / 2);
    focusX = wxMax(0, wxMin(focusX, m_frameWidth - focusSize));
    focusY = wxMax(0, focusY - focusSize / 2);
    focusY = wxMax(0, wxMin(focusY, m_frameHeight - focusSize));
    Rect focusSubFrame = Rect(focusX, focusY, focusSize, focusSize);
    focusRoi = FullFrame(focusSubFrame);

    meanSignal = cv::mean(focusRoi);
    scaleFactor = meanSignal[0] ? (65536.0 / 256) / meanSignal[0] : 1.0;

    focusRoi.convertTo(focusRoi, CV_32F, scaleFactor);
    return ComputeSobelSharpness(focusRoi);
}

// Get current detection status
void SolarSystemObject::GetDetectionStatus(wxString& statusMsg)
{
    if (GetPlanetDetectMode() == DETECTION_MODE_SURFACE)
        statusMsg = wxString::Format(_("Object at (%.1f, %.1f)"), m_center_x, m_center_y);
    else
        statusMsg = wxString::Format(_("Object at (%.1f, %.1f) radius=%d"), m_center_x, m_center_y, m_radius);
}

static double SiderealRateFromGuideSpeed(double guideSpeed)
{
    double const siderealSecsPerSec = 0.9973;
    return guideSpeed * 3600.0 / (15.0 * siderealSecsPerSec);
}

// Get the best estimate for the current mount guide speed and mount declination
bool SolarSystemObject::GetCalSettings(double *dec, double *speed)
{
    double sidRate = 0.5;
    double declination = 0.0;

    if (pPointingSource && pPointingSource->IsConnected())
    {
        double ra_val, dec_val, st;
        if (!pPointingSource->GetCoordinates(&ra_val, &dec_val, &st))
            declination = dec_val;

        double raSpd, decSpd;
        if (!pPointingSource->GetGuideRates(&raSpd, &decSpd) && pPointingSource->ValidGuideRates(raSpd, decSpd))
        {
            double minSpd = (decSpd != -1) ? wxMin(raSpd, decSpd) : raSpd;
            sidRate = SiderealRateFromGuideSpeed(minSpd);
        }
        else
        {
            CalibrationDetails calDetails;
            TheScope()->LoadCalibrationDetails(&calDetails);
            if (calDetails.IsValid() && TheScope()->ValidGuideRates(calDetails.raGuideSpeed, calDetails.decGuideSpeed))
                sidRate = SiderealRateFromGuideSpeed(wxMin(calDetails.raGuideSpeed, calDetails.decGuideSpeed));
        }
    }
    *speed = sidRate;
    *dec = declination;
    return true;
}

// Set system time lapse (msec)
void SolarSystemObject::SetTimeLapse(int msec)
{
    msec = wxMin(msec, 60000);
    msec = wxMax(msec, 0);
    pFrame->SetTimeLapse(msec);
}

// Update state used to visualize internally detected features
void SolarSystemObject::ShowVisualElements(bool state)
{
    m_syncLock.Lock();
    m_diskContour.clear();
    m_surf.inlierPoints.clear();
    m_showVisualElements = state;
    if (state == false)
        m_showMinMaxDiameters = false;
    m_syncLock.Unlock();
}

// Notification callback when PHD2 may change CaptureActive state
bool SolarSystemObject::UpdateCaptureState(bool CaptureActive)
{
    bool need_update = false;
    if (m_prevCaptureActive != CaptureActive)
    {
        if (!CaptureActive)
        {
            // Clear selection symbols (green circle/target lock) and visual elements
            if (Get_SolarSystemObjMode())
            {
                ShowVisualElements(false);
                pFrame->pGuider->Reset(false);
            }
            m_surf.guidingFixationPointValid = false;
            need_update = true;
        }
        else
        {
            // In solar/planetary mode update the state used to
            // control drawing of the internal detection elements.
            if (Get_SolarSystemObjMode() && GetShowFeaturesButtonState())
                ShowVisualElements(true);
            RestartSimulatorErrorDetection();
        }
    }

    // Reset the detection paused state if guiding has been cancelled
    if (!pFrame->pGuider->IsGuiding())
    {
        SetDetectionPausedState(false);
    }

    m_prevCaptureActive = CaptureActive;
    return need_update;
}

// Notification callback when camera is connected/disconnected
void SolarSystemObject::NotifyCameraConnect(bool connected)
{
    bool isSimCam = (pCamera && pCamera->Name == "Simulator");
    pFrame->pStatsWin->ShowSimulatorStats(isSimCam && connected);
    pFrame->pStatsWin->ShowPlanetStats(Get_SolarSystemObjMode() && connected);
    m_userLClick = false;
}

void SolarSystemObject::SaveCameraSimulationMove(double rx, double ry)
{
    m_cameraSimulationMove = Point2f(rx, ry);
    if (m_simulationZeroOffset)
    {
        m_cameraSimulationRefPoint = m_cameraSimulationMove;
        m_cameraSimulationRefPointValid = true;
    }
}

void SolarSystemObject::RestartSimulatorErrorDetection()
{
    m_cameraSimulationRefPointValid = false;
    m_simulationZeroOffset = true;
}

// Return scaled tracking image with lock target symbol
PHD_Point SolarSystemObject::GetScaledTracker(wxBitmap& scaledBitmap, const PHD_Point& star, double scale)
{
    // Select tracking symbol based on tracking quality
    int targetWidth;
    int targetHeight;
    wxImage *targetImage;
    if (m_surf.trackingQuality > 0.01)
    {
        targetImage = &m_lockTargetImageOk;
        targetWidth = m_lockTargetWidthOk;
        targetHeight = m_lockTargetHeightOk;
    }
    else
    {
        targetImage = &m_lockTargetImageBad;
        targetWidth = m_lockTargetWidthBad;
        targetHeight = m_lockTargetHeightBad;
    }

    // Limit size of lock target symbol
    int scaledWidth = (int) (targetWidth * scale / 5.0);
    int scaledHeight = (int) (targetHeight * scale / 5.0);
    scaledWidth = wxMin(scaledWidth, 160);
    scaledWidth = wxMax(scaledWidth, 96);
    scaledHeight = wxMin(scaledHeight, 160);
    scaledHeight = wxMax(scaledHeight, 96);

    // Enforce original aspect ratio before scaling
    if (targetWidth > targetHeight)
        scaledHeight = scaledWidth * targetHeight / targetWidth;
    else
        scaledWidth = scaledHeight * targetWidth / targetHeight;

    // Don't scale if the cached image is already scaled with the same parameters
    if ((scaledWidth != m_cachedScaledWidth) || (scaledHeight != m_cachedScaledHeight) || (targetImage != m_cachedTrackerImage))
    {
        wxImage scaledTrackerImage = targetImage->Scale(scaledWidth, scaledHeight, wxIMAGE_QUALITY_HIGH);
        m_cachedTrackerScaledBitmap = wxBitmap(scaledTrackerImage);
        m_cachedScaledWidth = scaledWidth;
        m_cachedScaledHeight = scaledHeight;
        m_cachedTrackerImage = targetImage;
    }

    scaledBitmap = m_cachedTrackerScaledBitmap;
    return PHD_Point(star.X * scale - scaledWidth / 2, star.Y * scale - scaledHeight / 2);
}

// Helper for visualizing detection radius and internal features
void SolarSystemObject::VisualHelper(wxDC& dc, Star primaryStar, double scaleFactor)
{
    // Do nothin if not in solar/planetary mode or no visual elements are enabled
    if (!Get_SolarSystemObjMode() || !m_showMinMaxDiameters && !VisualElementsEnabled())
        return;

    // Clip drawing region to displayed image frame
    wxImage *pImg = pFrame->pGuider->DisplayedImage();
    if (pImg)
        dc.SetClippingRegion(wxRect(0, 0, pImg->GetWidth(), pImg->GetHeight()));

    // Make sure to use transparent brush
    dc.SetBrush(*wxTRANSPARENT_BRUSH);

    // Display internally detected elements (must be enabled in UI)
    if (VisualElementsEnabled())
    {
        m_syncLock.Lock();

        switch (GetPlanetDetectMode())
        {
        case DETECTION_MODE_SURFACE:
            // Draw detected surface features
            dc.SetPen(wxPen(wxColour(230, 0, 0), 2, wxPENSTYLE_SOLID));
            for (const auto& feature : m_surf.inlierPoints)
            {
                dc.DrawCircle(feature.x * scaleFactor, feature.y * scaleFactor, 5);
            }
            break;
        case DETECTION_MODE_DISK:
            // Draw contour points in solar/planetary mode
            if (m_diskContour.size())
            {
                dc.SetPen(wxPen(wxColour(230, 0, 0), 2, wxPENSTYLE_SOLID));
                for (const Point2f& contourPoint : m_diskContour)
                    dc.DrawCircle((contourPoint.x + m_roiRect.x) * scaleFactor, (contourPoint.y + m_roiRect.y) * scaleFactor,
                                  2);

#ifdef DEVELOPER_MODE
                // Mark positions of detected centroid and smallest enclosing circle centers - in simulator mode only
                if (pCamera && pCamera->Name == "Simulator")
                {
                    // Draw anchor circle centers
                    dc.SetLogicalFunction(wxXOR);
                    dc.SetPen(wxPen(wxColour(230, 230, 0), 3, wxPENSTYLE_SOLID));
                    if (m_centoid_x && m_centoid_y)
                        dc.DrawCircle((m_centoid_x + m_roiRect.x) * scaleFactor, (m_centoid_y + m_roiRect.y) * scaleFactor, 3);
                    dc.SetPen(wxPen(wxColour(230, 230, 0), 1, wxPENSTYLE_SOLID));
                    if (m_sm_circle_x && m_sm_circle_y)
                        dc.DrawCircle((m_sm_circle_x + m_roiRect.x) * scaleFactor, (m_sm_circle_y + m_roiRect.y) * scaleFactor,
                                      3);
                    dc.SetLogicalFunction(wxCOPY);
                }
#endif
            }
            break;
        }

        m_syncLock.Unlock();
    }

    // Reset clipping region (don't clip min/max circles)
    dc.DestroyClippingRegion();

    // Display min/max diameters for visual feedback
    if (m_showMinMaxDiameters)
    {
        if (m_DiameterStopWatch.Time() > 5000)
            m_showMinMaxDiameters = false;
        if (!GetSurfaceTrackingState() && pFrame->CaptureActive)
        {
            const wxString labelTextMin("min diameter");
            const wxString labelTextMax("max diameter");
            int x = int(primaryStar.X * scaleFactor + 0.5);
            int y = int(primaryStar.Y * scaleFactor + 0.5);
            int radius = int(m_radius * scaleFactor + 0.5);
            float minRadius = Get_minRadius() * scaleFactor;
            float maxRadius = Get_maxRadius() * scaleFactor;
            int minRadius_x = x + minRadius;
            int maxRadius_x = x + maxRadius;
            int lineMin_x = x;
            int lineMax_x = x;

            // Center the elements at the tracking point
            if (m_detected)
            {
                minRadius_x = maxRadius_x = x;
                lineMin_x -= minRadius;
                lineMax_x -= maxRadius;
            }

            // Draw min and max diameters legends
            dc.SetPen(wxPen(wxColour(230, 130, 30), 1, wxPENSTYLE_DOT));
            dc.SetTextForeground(wxColour(230, 130, 30));
            dc.DrawLine(lineMin_x, y - 5, lineMin_x + minRadius * 2, y - 5);
            dc.DrawCircle(minRadius_x, y, minRadius);
            dc.DrawText(labelTextMin, minRadius_x - dc.GetTextExtent(labelTextMin).GetWidth() / 2,
                        y - 10 - dc.GetTextExtent(labelTextMin).GetHeight());

            dc.SetPen(wxPen(wxColour(130, 230, 30), 1, wxPENSTYLE_DOT));
            dc.SetTextForeground(wxColour(130, 230, 30));
            dc.DrawLine(lineMax_x, y + 5, lineMax_x + maxRadius * 2, y + 5);
            dc.DrawCircle(maxRadius_x, y, maxRadius);
            dc.DrawText(labelTextMax, maxRadius_x - dc.GetTextExtent(labelTextMax).GetWidth() / 2, y + 5);
        }
    }
}

void SolarSystemObject::CalcLineParams(CircleDescriptor p1, CircleDescriptor p2)
{
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    if ((p1.radius == 0) || (p2.radius == 0) || (dx * dx + dy * dy < 3))
    {
        m_DiameterLineParameters.valid = false;
        m_DiameterLineParameters.vertical = false;
        m_DiameterLineParameters.slope = 0;
        m_DiameterLineParameters.b = 0;
        return;
    }
    // Check to see if line is vertical
    if (fabs(p1.x - p2.x) < 1)
    {
        // Vertical line, slope is undefined
        m_DiameterLineParameters.valid = true;
        m_DiameterLineParameters.vertical = true;
        m_DiameterLineParameters.slope = std::numeric_limits<double>::infinity();
        m_DiameterLineParameters.b = 0;
    }
    else
    {
        // Calculate slope (m) and y-intercept (b) for a non-vertical line
        m_DiameterLineParameters.valid = true;
        m_DiameterLineParameters.vertical = false;
        m_DiameterLineParameters.slope = (p2.y - p1.y) / (p2.x - p1.x);
        m_DiameterLineParameters.b = p1.y - (m_DiameterLineParameters.slope * p1.x);
    }
}

// Calculate score for given point
static float CalcContourScore(float& radius, Point2f pointToMeasure, std::vector<Point2f>& diskContour, int minRadius,
                              int maxRadius)
{
    std::vector<float> distances;
    distances.reserve(diskContour.size());
    float minIt = FLT_MAX;
    float maxIt = FLT_MIN;

    for (const auto& contourPoint : diskContour)
    {
        float distance = norm(contourPoint - pointToMeasure);
        if (distance >= minRadius && distance <= maxRadius)
        {
            minIt = wxMin(minIt, distance);
            maxIt = wxMax(maxIt, distance);
            distances.push_back(distance);
        }
    }

    // Note: calculating histogram on 0-sized data can crash the application.
    // Reject small sets of points as they usually aren't related to the features we are looking for.
    if (distances.size() < 16)
    {
        radius = 0;
        return 0;
    }

    // Calculate the number of bins
    int bins = int(std::sqrt(distances.size()) + 0.5) | 1;
    float range[] = { std::floor(minIt), std::ceil(maxIt) };
    const float *histRange[] = { range };

    // Calculate the histogram
    Mat hist;
    Mat distData(distances); // Use vector directly to create Mat object
    cv::calcHist(&distData, 1, nullptr, Mat(), hist, 1, &bins, histRange, true, false);

    // Find the peak of the histogram
    double max_value;
    Point max_loc;
    cv::minMaxLoc(hist, nullptr, &max_value, nullptr, &max_loc);
    int max_idx = max_loc.y;

    // Middle of the bin
    float peakDistance = range[0] + (max_idx + 0.5) * ((range[1] - range[0]) / bins);

    float scorePoints = 0;
    for (float distance : distances)
    {
        int index = fabs(distance - peakDistance) * 100 + 0.5;
        if (index < GAUSSIAN_SIZE)
            scorePoints += gaussianWeight[index];
    }

    // Normalize score by total number points in the contour
    radius = peakDistance;
    return scorePoints / diskContour.size();
}

class AsyncCalcScoreThread : public wxThread
{
public:
    std::vector<Point2f> points;
    std::vector<Point2f> contour;
    Point2f center;
    float radius;
    float threadBestScore;
    int minRadius;
    int maxRadius;

public:
    AsyncCalcScoreThread(float bestScore, std::vector<Point2f>& diskContour, std::vector<Point2f>& workLoad, int min_radius,
                         int max_radius)
        : wxThread(wxTHREAD_JOINABLE), threadBestScore(bestScore), contour(diskContour), points(workLoad),
          minRadius(min_radius), maxRadius(max_radius)
    {
        radius = 0;
    }
    // A thread function to run HoughCircles method
    wxThread::ExitCode Entry()
    {
        for (const Point2f& point : points)
        {
            float score = ::CalcContourScore(radius, point, contour, minRadius, maxRadius);
            if (score > threadBestScore)
            {
                threadBestScore = score;
                radius = radius;
                center.x = point.x;
                center.y = point.y;
            }
        }
        return this;
    }
};

/* Find best circle candidate */
int SolarSystemObject::RefineDiskCenter(float& bestScore, CircleDescriptor& diskCenter, std::vector<Point2f>& diskContour,
                                        int minRadius, int maxRadius, float searchRadius, float resolution)
{
    const int maxWorkloadSize = 256;
    const Point2f center = { diskCenter.x, diskCenter.y };
    std::vector<AsyncCalcScoreThread *> threads;

    // Check all points within small circle for search of higher score
    int threadCount = 0;
    bool useThreads = true;
    int workloadSize = 0;
    std::vector<Point2f> workload;
    workload.reserve(maxWorkloadSize);
    for (float x = diskCenter.x - searchRadius; x < diskCenter.x + searchRadius; x += resolution)
        for (float y = diskCenter.y - searchRadius; y < diskCenter.y + searchRadius; y += resolution)
        {
            Point2f pointToMeasure = { x, y };
            float dist = norm(pointToMeasure - center);
            if (dist > searchRadius)
                continue;

            // When finished creating a workload, create and run new processing thread
            if (useThreads && (workloadSize++ >= maxWorkloadSize))
            {
                AsyncCalcScoreThread *thread = new AsyncCalcScoreThread(bestScore, diskContour, workload, minRadius, maxRadius);
                if ((thread->Create() == wxTHREAD_NO_ERROR) && (thread->Run() == wxTHREAD_NO_ERROR))
                {
                    threads.push_back(thread);
                    workload.clear();
                    workloadSize = 0;
                    threadCount++;
                }
                else
                {
                    useThreads = false;
                    Debug.Write(_("RefineDiskCenter: failed to start a thread\n"));
                }
            }
            workload.push_back(pointToMeasure);
        }

    // Process remaining points locally
    for (const Point2f& point : workload)
    {
        float radius;
        float score = ::CalcContourScore(radius, point, diskContour, minRadius, maxRadius);
        if (score > bestScore)
        {
            bestScore = score;
            diskCenter.radius = radius;
            diskCenter.x = point.x;
            diskCenter.y = point.y;
        }
    }

    // Wait for all threads to terminate and process their results
    for (auto thread : threads)
    {
        thread->Wait();
        if (thread->threadBestScore > bestScore)
        {
            bestScore = thread->threadBestScore;
            diskCenter.radius = thread->radius;
            diskCenter.x = thread->center.x;
            diskCenter.y = thread->center.y;
        }

        delete thread;
    }
    return threadCount;
}

// An algorithm to find contour center
float SolarSystemObject::FindContourCenter(CircleDescriptor& diskCenter, CircleDescriptor& circle,
                                           std::vector<Point2f>& diskContour, Moments& mu, int minRadius, int maxRadius)
{
    float score;
    float maxScore = 0;
    float bestScore = 0;
    float radius = 0;
    int searchRadius = circle.radius / 2;
    Point2f pointToMeasure;
    std::vector<WeightedCircle> WeightedCircles;
    WeightedCircles.reserve(searchRadius * 2);

    // When center of mass (centroid) wasn't found use smallest circle for measurement
    if (!m_DiameterLineParameters.valid)
    {
        pointToMeasure.x = circle.x;
        pointToMeasure.y = circle.y;
        score = CalcContourScore(radius, pointToMeasure, diskContour, minRadius, maxRadius);
        diskCenter = circle;
        diskCenter.radius = radius;
        return score;
    }

    if (!m_DiameterLineParameters.vertical && (fabs(m_DiameterLineParameters.slope) <= 1.0))
    {
        // Search along x-axis when line slope is below 45 degrees
        for (pointToMeasure.x = circle.x - searchRadius; pointToMeasure.x <= circle.x + searchRadius; pointToMeasure.x++)
        {
            // Count number of points of the contour which are equidistant from pointToMeasure.
            // The point with maximum score is identified as contour center.
            pointToMeasure.y = m_DiameterLineParameters.slope * pointToMeasure.x + m_DiameterLineParameters.b;
            score = CalcContourScore(radius, pointToMeasure, diskContour, minRadius, maxRadius);
            maxScore = max(score, maxScore);
            WeightedCircle wcircle = { pointToMeasure.x, pointToMeasure.y, radius, score };
            WeightedCircles.push_back(wcircle);
        }
    }
    else
    {
        // Search along y-axis when slope is above 45 degrees
        for (pointToMeasure.y = circle.y - searchRadius; pointToMeasure.y <= circle.y + searchRadius; pointToMeasure.y++)
        {
            // Count number of points of the contour which are equidistant from pointToMeasure.
            // The point with maximum score is identified as contour center.
            if (m_DiameterLineParameters.vertical)
                pointToMeasure.x = circle.x;
            else
                pointToMeasure.x = (pointToMeasure.y - m_DiameterLineParameters.b) / m_DiameterLineParameters.slope;
            score = CalcContourScore(radius, pointToMeasure, diskContour, minRadius, maxRadius);
            maxScore = max(score, maxScore);
            WeightedCircle wcircle = { pointToMeasure.x, pointToMeasure.y, radius, score };
            WeightedCircles.push_back(wcircle);
        }
    }

    // Find local maxima point closer to center of mass,
    // this will help not to select center of the dark disk
    int bestIndex = 0;
    float bestCenterOfMassDistance = 999999;
    Point2f centroid = { float(mu.m10 / mu.m00), float(mu.m01 / mu.m00) };
    for (int i = 1; i < WeightedCircles.size() - 1; i++)
    {
        if ((WeightedCircles[i].score > maxScore * 0.65) && (WeightedCircles[i].score > WeightedCircles[i - 1].score) &&
            (WeightedCircles[i].score > WeightedCircles[i + 1].score))
        {
            WeightedCircle *localMax = &WeightedCircles[i];
            Point2f center = { localMax->x, localMax->y };
            float centerOfMassDistance = norm(centroid - center);
            if (centerOfMassDistance < bestCenterOfMassDistance)
            {
                bestCenterOfMassDistance = centerOfMassDistance;
                bestIndex = i;
            }
        }
    }
    if (WeightedCircles.size() < 3)
    {
        for (int i = 0; i < WeightedCircles.size(); i++)
            if (WeightedCircles[i].score > bestScore)
            {
                bestScore = WeightedCircles[i].score;
                bestIndex = i;
            }
    }

    bestScore = WeightedCircles[bestIndex].score;
    diskCenter.radius = WeightedCircles[bestIndex].r;
    diskCenter.x = WeightedCircles[bestIndex].x;
    diskCenter.y = WeightedCircles[bestIndex].y;

    return bestScore;
}

// Find a minimum enclosing circle of the contour and also its center of mass
void SolarSystemObject::FindCenters(Mat image, const std::vector<Point>& contour, CircleDescriptor& centroid,
                                    CircleDescriptor& circle, std::vector<Point2f>& diskContour, Moments& mu, int minRadius,
                                    int maxRadius)
{
    const std::vector<Point> *effectiveContour = &contour;
    std::vector<Point> decimatedContour;
    Point2f circleCenter;
    float circle_radius = 0;

    // Add extra margins for min/max radii allowing inclusion of contours
    // outside and inside the given range.
    maxRadius = (maxRadius * 5) / 4;
    minRadius = (minRadius * 3) / 4;

    m_eccentricity = 0;
    m_angle = 0;
    circle.radius = 0;
    centroid.radius = 0;
    diskContour.clear();

    // If input contour is too large, decimate it to avoid performance issues
    int decimateRatio = contour.size() > 4096 ? contour.size() / 4096 : 1;
    if (decimateRatio > 1)
    {
        decimatedContour.reserve(contour.size() / decimateRatio);
        for (int i = 0; i < contour.size(); i += decimateRatio)
            decimatedContour.push_back(contour[i]);
        effectiveContour = &decimatedContour;
    }
    diskContour.reserve(effectiveContour->size());
    minEnclosingCircle(*effectiveContour, circleCenter, circle_radius);

    if ((circle_radius <= maxRadius) && (circle_radius >= minRadius))
    {
        // Convert contour to vector of floating points
        for (int i = 0; i < effectiveContour->size(); i++)
        {
            Point pt = (*effectiveContour)[i];
            diskContour.push_back(Point2f(pt.x, pt.y));
        }

        circle.x = circleCenter.x;
        circle.y = circleCenter.y;
        circle.radius = circle_radius;

        // Calculate center of mass based on contour points
        mu = cv::moments(diskContour, false);
#if 0
        // Calculate center of mass based on original image masked by the circle
        Mat maskedImage;
        Mat mask = Mat::zeros(image.size(), CV_8U);
        Point center(circle.x, circle.y);
        cv::circle(mask, center, circle_radius, 255, -1);
        bitwise_and(image, mask, maskedImage);
        mu = cv::moments(maskedImage, false);
#endif
        if (mu.m00 > 0)
        {
            centroid.x = mu.m10 / mu.m00;
            centroid.y = mu.m01 / mu.m00;
            centroid.radius = circle.radius;

            // Calculate eccentricity
            double a = mu.mu20 + mu.mu02;
            double b = sqrt(4 * mu.mu11 * mu.mu11 + (mu.mu20 - mu.mu02) * (mu.mu20 - mu.mu02));
            double major_axis = sqrt(2 * (a + b));
            double minor_axis = sqrt(2 * (a - b));
            m_eccentricity = sqrt(1 - (minor_axis * minor_axis) / (major_axis * major_axis));

            // Calculate orientation (theta) in radians and convert to degrees
            float theta = 0.5 * atan2(2 * mu.mu11, (mu.mu20 - mu.mu02));
            m_angle = theta * (180.0 / CV_PI);
        }
    }
}

// Calculate position of fixation point
Point2f SolarSystemObject::calculateCentroid(const std::vector<KeyPoint>& keypoints, Point2f& clickedPoint)
{
    // If no clicked point is available, calculate centroid of keypoints
    if ((clickedPoint.x == 0) || (clickedPoint.y == 0) || (clickedPoint.x > m_frameWidth) || (clickedPoint.y > m_frameHeight))
    {
        // Calculate centroid of keypoints to be used as a virtual tracking point
        Point2f sum(0, 0);
        for (const auto& kp : keypoints)
            sum += kp.pt;
        // Radius affects scaling factor for the Star Profile window
        m_radius = m_starProfileSize;
        return Point2f(sum.x / keypoints.size(), sum.y / keypoints.size());
    }
    else
    {
        // Find closest keypoint to clicked point
        double minDist = 999999;
        KeyPoint trackedKeypoint;
        Point2f closestPoint = Point2f(0, 0);
        for (const auto& kp : keypoints)
        {
            // Calculate distance between clicked point and keypoint but avoid points too close to the edge
            const int margin = 50;
            double dist = norm(kp.pt - clickedPoint);
            if ((dist < minDist) && (kp.pt.x > 50) && (kp.pt.y > 50) && (kp.pt.x < m_frameWidth - 50) &&
                (kp.pt.y < m_frameHeight - 50))
            {
                minDist = dist;
                trackedKeypoint = kp;
                closestPoint = kp.pt;
            }
        }
        m_radius = m_starProfileSize;
        return closestPoint;
    }
}

// Function to check for collinearity
bool SolarSystemObject::areCollinear(const KeyPoint& kp1, const KeyPoint& kp2, const KeyPoint& kp3)
{
    double area2 = abs((kp2.pt.x - kp1.pt.x) * (kp3.pt.y - kp1.pt.y) - (kp3.pt.x - kp1.pt.x) * (kp2.pt.y - kp1.pt.y));
    return area2 < 2.0; // Consider using a relative threshold based on image dimensions
}

// Function to validate and filter keypoints
bool SolarSystemObject::validateAndFilterKeypoints(std::vector<KeyPoint>& keypoints, std::vector<KeyPoint>& filteredKeypoints,
                                                   int maxKeypoints)
{
    std::vector<KeyPoint> selectedKeypoints;
    selectedKeypoints.reserve(keypoints.size());
    filteredKeypoints.clear();

    // Check for sufficient keypoints (at least 4)
    if (keypoints.size() < 4)
    {
        return false;
    }

    // Filter coincident points
    double dist_threshold = HOMOGRAPHY_DIST_THRESHOLD;
    for (size_t i = 0; i < keypoints.size(); ++i)
    {
        bool unique = true;
        for (size_t j = 0; j < i; ++j)
        {
            if (norm(keypoints[i].pt - keypoints[j].pt) < dist_threshold)
            {
                unique = false;
                break;
            }
        }
        if (unique)
            selectedKeypoints.push_back(keypoints[i]);
    }

    // Filter collinear points
    int size = selectedKeypoints.size();
    for (size_t i = 0; i < size; ++i)
    {
        for (size_t j = i + 1; j < size; ++j)
        {
            for (size_t k = j + 1; k < size; ++k)
            {
                if (areCollinear(selectedKeypoints[i], selectedKeypoints[j], selectedKeypoints[k]))
                {
                    selectedKeypoints.erase(selectedKeypoints.begin() + k);
                    --k; // Adjust index after erasing
                    --size;
                }
            }
        }
    }

    // Limit to top N keypoints
    if (maxKeypoints == 0)
        maxKeypoints = PT_MAX_SURFACE_FEATURES;
    maxKeypoints = wxMin(maxKeypoints, (int) selectedKeypoints.size());
    filteredKeypoints.assign(selectedKeypoints.begin(), selectedKeypoints.begin() + maxKeypoints);

    // Check for sufficient keypoints (at least 4)
    return (filteredKeypoints.size() < 4) ? false : true;
}

void SolarSystemObject::Set_minHessian(int value)
{
    value = wxMax(PT_MIN_HESSIAN_UI_MIN, wxMin(value, PT_MIN_HESSIAN_UI_MAX));
    if (m_paramMinHessian != value)
    {
        m_paramMinHessian = value;
        m_surfaceDetectionParamsChanging = true;
    }
}

void SolarSystemObject::Set_maxFeatures(int value)
{
    value = wxMax(PT_MIN_SURFACE_FEATURES, wxMin(value, PT_MAX_SURFACE_FEATURES));
    if (m_paramMaxFeatures != value)
    {
        m_paramMaxFeatures = value;
        m_surfaceDetectionParamsChanging = true;
    }
}

int SolarSystemObject::Get_minHessian()
{
    return wxMax(PT_MIN_HESSIAN_UI_MIN, wxMin(m_paramMinHessian, PT_MIN_HESSIAN_UI_MAX));
}

// Map the slider value to physical minHessian parameter value using inverse logarithmic scale
int SolarSystemObject::Get_minHessianPhysical()
{
    // Ensure the sensitivity value is within the expected range
    int uiSensitivityValue = Get_minHessian();
    uiSensitivityValue = wxMax(PT_MIN_HESSIAN_UI_MIN, wxMin(uiSensitivityValue, PT_MIN_HESSIAN_UI_MAX));

    // Calculate the inverse ratio of the current position to the maximum UI value
    double ratio = (static_cast<double>(uiSensitivityValue) / static_cast<double>(PT_MIN_HESSIAN_UI_MAX));

    // Apply an inverse logarithmic scale to the ratio
    double logRatio = log2(1 + (ratio * 1023));

    // Map the logarithmic ratio to the minHessian range inversely
    int mappedValue = static_cast<int>(PT_MIN_HESSIAN_MAX - ((logRatio / 10.0) * (PT_MIN_HESSIAN_MAX - PT_MIN_HESSIAN_MIN)));

    return mappedValue;
}

// Detect/track surface features
bool SolarSystemObject::DetectSurfaceFeatures(Mat image, Point2f& clickedPoint, bool autoSelect)
{
    // No detected features yet
    m_detectedFeatures = 0;

    // Search region for star find is fixed value for surface features tracking
    m_searchRegion = 128;

    // Variance is not known yet
    m_surf.variance = 0.0;

    // Assume this won't be a reference frame
    m_surf.isReferenceFrame = false;

    // Enhance local contrast before feature detection
    m_surf.clahe->apply(image, m_surf.equalized);

    // Create SURF detector
    int nOctaves = 4;
    int nOctaveLayers = 2;
    bool upright = true;
    bool surfExtended = false;
    m_surf.surfDetector = SurfFeatureDetector(Get_minHessianPhysical(), nOctaves, nOctaveLayers, surfExtended, upright);

    // Detect keypoints
    std::vector<KeyPoint> keypoints;
    m_surf.surfDetector.detect(m_surf.equalized, keypoints);

    // Set locked guiding position when guiding starts
    if (pFrame->pGuider->IsGuiding())
    {
        if (!m_surf.guidingFixationPointValid && m_surf.referenceKeypoints.size())
        {
            m_surf.guidingFixationPoint = m_surf.surfaceFixationPoint;
            m_surf.guidingFixationPointValid = true;
        }
    }
    else
    {
        // While not guiding we can still reset the fixation point based on new clicked point or autoselect
        if (autoSelect || (clickedPoint != m_prevClickedPoint))
            m_surf.referenceKeypoints.clear();
        m_surf.guidingFixationPointValid = false;
    }

    // Exclude keypoints which are too close to frame edges.
    // When setting the reference frame, we limit keypoints to be further away from the edges.
    const int edgeMargin = m_surf.referenceKeypoints.size() ? 15 : 30;
    for (auto it = keypoints.begin(); it != keypoints.end();)
    {
        if ((it->pt.x < edgeMargin) || (it->pt.y < edgeMargin) || (it->pt.x > m_frameWidth - edgeMargin) ||
            (it->pt.y > m_frameHeight - edgeMargin))
            it = keypoints.erase(it);
        else
            ++it;
    }

    // Limit to top N keypoints
    int maxKeypoints =
        wxMin(m_surf.referenceKeypoints.size() ? m_paramMaxFeatures : PT_MAX_SURFACE_FEATURES, (int) keypoints.size());
    std::vector<KeyPoint> topKeypoints;
    if (keypoints.size() <= maxKeypoints)
        topKeypoints = keypoints;
    else
    {
        // Sort keypoints by response and limit to top N
        std::sort(keypoints.begin(), keypoints.end(),
                  [](const KeyPoint& a, const KeyPoint& b) { return a.response > b.response; });
        topKeypoints.assign(keypoints.begin(), keypoints.begin() + maxKeypoints);
    }

    // Filter keypoints and limit by the maximum number of features
    if (!validateAndFilterKeypoints(topKeypoints, m_surf.filteredKeypoints, maxKeypoints))
    {
        // Indicate insufficient keypoints
        m_statusMsg = _("No detectable features");
        Debug.Write("Surface feature tracking: " + m_statusMsg + "\n");
        return false;
    }

    // Extract descriptors
    m_surf.extractor.compute(m_surf.equalized, m_surf.filteredKeypoints, m_surf.descriptors);

    // When reference keypoints are available, filter and validate keypoints
    if (m_surf.referenceKeypoints.size())
    {
        // Match descriptors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher.knnMatch(m_surf.referenceDescriptors, m_surf.descriptors, knnMatches, 2);

        // Lowe's Ratio Test : https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
        // This helps to ensure that matches are distinct and likely to be correct.
        std::vector<DMatch> matches;
        matches.reserve(m_surf.descriptors.rows);
        const float ratio_thresh = 0.75;
        for (size_t i = 0; i < knnMatches.size(); i++)
        {
            if (knnMatches[i].size() == 2 && knnMatches[i][0].distance < ratio_thresh * knnMatches[i][1].distance)
                matches.push_back(knnMatches[i][0]);
        }

        // A minimum number of 4 points is required to find RANSAC homography
        if (matches.size() < 4)
        {
            m_statusMsg = _("Too few matched features");
            Debug.Write("Surface feature tracking: " + m_statusMsg + "\n");
            return false;
        }

        // Extract location of good matches
        std::vector<Point2f> points1;
        std::vector<Point2f> points2;
        points1.reserve(matches.size());
        points2.reserve(matches.size());
        for (const DMatch& match : matches)
        {
            points1.push_back(m_surf.referenceKeypoints[match.queryIdx].pt);
            points2.push_back(m_surf.filteredKeypoints[match.trainIdx].pt);
        }

        // Find homography using RANSAC
        Mat mask; // This will be filled with the inliers mask
        double ransacReprojThreshold = HOMOGRAPHY_DIST_THRESHOLD;
        Mat H = findHomography(points1, points2, CV_RANSAC, ransacReprojThreshold, mask);

        // Use the mask to filter out the outliers
        Point2f displacement(0, 0);
        std::vector<DMatch> inlierMatches;
        std::vector<Point2f> inlierPoints;
        std::vector<double> distances;
        inlierMatches.reserve(mask.rows);
        inlierPoints.reserve(mask.rows);
        distances.reserve(mask.rows);
        for (size_t i = 0; i < mask.rows; i++)
        {
            if (mask.at<unsigned char>(i))
            {
                auto match = matches[i];
                inlierMatches.push_back(match);
                inlierPoints.push_back(m_surf.filteredKeypoints[match.trainIdx].pt);

                // Compute average displacement vector between matched keypoints
                displacement += m_surf.filteredKeypoints[match.trainIdx].pt - m_surf.referenceKeypoints[match.queryIdx].pt;

                // Calculate distances between matched descriptors
                double dist = norm(m_surf.referenceKeypoints[match.queryIdx].pt - m_surf.filteredKeypoints[match.trainIdx].pt);
                distances.push_back(dist);
            }
        }

        // Discard if very few inliers were found
        if (inlierMatches.size() < 4)
        {
            m_statusMsg = _("Too few detectable features");
            Debug.Write("Surface feature tracking: " + m_statusMsg + "\n");
            return false;
        }

        // Calculate mean, variance, and score
        double mean = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        m_surf.variance = 0.0;
        for (double distance : distances)
        {
            double delta = distance - mean;
            m_surf.variance += delta * delta;
            mean += delta / distances.size(); // Update mean for unbiased estimator
        }
        m_surf.variance /= distances.size() - 1; // Adjust for unbiased estimator

        // If variance is too high, fail detection and don't update fixation point
        if (m_surf.variance > 100)
        {
            m_statusMsg = wxString::Format("Dispersion too high (features:%d)", inlierMatches.size());
            Debug.Write("Surface feature tracking: " + m_statusMsg + "\n");
            return false;
        }

        // Compute average displacement Point2f vector between matched keypoints
        displacement.x /= inlierMatches.size();
        displacement.y /= inlierMatches.size();

        // Calculate new position based on average displacement vector and virtual tracking point
        m_surf.surfaceFixationPoint = m_surf.referencePoint + displacement;

        // Compute distance from the locked guiding position
        double distance =
            m_surf.guidingFixationPointValid ? norm(m_surf.surfaceFixationPoint - m_surf.guidingFixationPoint) : 0;

        // Estimate tracking quality based on variance
        if (m_surf.variance <= 1.0)
        {
            m_surf.trackingQuality = 1.0;
        }
        else
        {
            const double k = 0.199715;
            m_surf.trackingQuality = std::exp(-k * (m_surf.variance - 1.0));
        }
        // Adjust tracking accuracy using the maximum permitted distance from the reference frame
        if (distance >= 100)
            m_surf.trackingQuality = 0;

        // If variance becomes too high, try to switch to the new reference frame
        const float varianceThreshold = 16.0;
        if ((m_surf.variance > varianceThreshold) || m_surfaceDetectionParamsChanging || m_forceReferenceFrameSwitch)
        {
            // Switch to red tracking box as a warning sign
            if (m_surf.trackingQuality < 0.25)
                m_surf.trackingQuality = 0;
            m_surfaceDetectionParamsChanging = false;

            // When guiding restrict switching to the new reference frame
            if (!pFrame->pGuider->IsGuiding() || m_forceReferenceFrameSwitch ||
                ((distance < 100) && (m_surf.variance < varianceThreshold * 2)))
            {
                // Reset flag after trying to switch to the new reference frame
                m_forceReferenceFrameSwitch = false;

                // When selecting new reference frame, recalculate keypoints and descriptors
                // without limiting the number of features.
                validateAndFilterKeypoints(topKeypoints, m_surf.filteredKeypoints, 0);

                // Exclude keypoints which are too close to frame edges.
                // When setting the reference frame, we limit keypoints to be further away from the edges.
                const int edgeMargin = 30;
                for (auto it = m_surf.filteredKeypoints.begin(); it != m_surf.filteredKeypoints.end();)
                {
                    if ((it->pt.x < edgeMargin) || (it->pt.y < edgeMargin) || (it->pt.x > m_frameWidth - edgeMargin) ||
                        (it->pt.y > m_frameHeight - edgeMargin))
                        it = m_surf.filteredKeypoints.erase(it);
                    else
                        ++it;
                }
                if (m_surf.filteredKeypoints.size() < 4)
                {
                    m_statusMsg = _("Too few detectable features");
                    Debug.Write("Surface feature tracking: " + m_statusMsg + "\n");
                    return false;
                }

                // We must recalculate descriptors for the filtered keypoints
                m_surf.extractor.compute(m_surf.equalized, m_surf.filteredKeypoints, m_surf.descriptors);

                // Set new reference frame. The new position may not be rather accurate
                m_surf.referencePoint = m_surf.surfaceFixationPoint;
                m_surf.referenceKeypoints = m_surf.filteredKeypoints;
                m_surf.referenceDescriptors = m_surf.descriptors.clone();
                m_surf.isReferenceFrame = true;
                Debug.Write("Surface feature tracking: update reference point\n");
            }

            // Warn user about unstable image
            if ((m_surf.variance > 100) && pFrame->pGuider->IsGuiding())
            {
                Debug.Write(
                    wxString::Format("Feature matching encountered very large variance (%.1f), position may not be accurate.\n",
                                     m_surf.variance));
                pFrame->Alert(_("WARNING: image is not stable, tracking may not be accurate!"), wxICON_WARNING);
            }
        }

        // Find the keypoint closest to the surface fixation point
        KeyPoint *trackedKeypoint = NULL;
        float trackedPosMinDistance = 999999.0;
        for (int i = 0; i < m_surf.filteredKeypoints.size(); ++i)
        {
            double dist = norm(m_surf.surfaceFixationPoint - m_surf.filteredKeypoints[i].pt);
            if (dist < trackedPosMinDistance)
            {
                trackedPosMinDistance = dist;
                trackedKeypoint = &m_surf.filteredKeypoints[i];
            }
        }
        m_radius = m_starProfileSize;

        // Save inlier matches for visualization
        if (VisualElementsEnabled())
        {
            m_syncLock.Lock();
            m_surf.inlierPoints = inlierPoints;
            m_syncLock.Unlock();
        }

        // Count detected features
        m_detectedFeatures = inlierPoints.size();

        // Update stats
        Debug.Write(
            wxString::Format("Surface feature tracking: mean=%.1f, variance=%.1f, distance=%.1f, quality=%.1f, features=%d\n",
                             mean, m_surf.variance, distance, m_surf.trackingQuality, m_detectedFeatures));
        pFrame->pStatsWin->UpdatePlanetScore(_T("Dispersion"), m_surf.variance);
    }
    // Set reference frame keypoints and descriptors
    else if (m_surf.descriptors.rows > 4)
    {
        // Tracking quality is not available yet
        m_surf.trackingQuality = 0;

        m_surf.referenceKeypoints = m_surf.filteredKeypoints;
        m_surf.referenceDescriptors = m_surf.descriptors.clone();
        m_surf.isReferenceFrame = true;

        // Save reference frame centroid
        m_surf.referencePoint = calculateCentroid(m_surf.filteredKeypoints, clickedPoint);
        m_surf.surfaceFixationPoint = m_surf.referencePoint;

        // Save reference keypoints for visualization
        if (VisualElementsEnabled())
        {
            m_syncLock.Lock();
            m_surf.inlierPoints.clear();
            m_surf.inlierPoints.reserve(m_surf.referenceKeypoints.size());
            for (const auto& kp : m_surf.referenceKeypoints)
                m_surf.inlierPoints.push_back(kp.pt);
            m_syncLock.Unlock();
        }

        // Count detected features
        m_detectedFeatures = m_surf.referenceKeypoints.size();

        // Assume no more changes to minHessian until further notice
        m_surfaceDetectionParamsChanging = false;
        Debug.Write(wxString::Format("Surface feature tracking: set new reference point, features=%d\n", m_detectedFeatures));
    }

    // Set new object position based on updated centroid
    m_center_x = m_surf.surfaceFixationPoint.x;
    m_center_y = m_surf.surfaceFixationPoint.y;

    return true;
}

// Find orb center using circle matching with contours
bool SolarSystemObject::FindOrbisCenter(Mat img8, int minRadius, int maxRadius, bool roiActive, Point2f& clickedPoint,
                                        Rect& roiRect, bool activeRoiLimits, float distanceRoiMax)
{
    m_planetaryContourPoints = 0;
    m_planetaryFittingScore = 0;

    int LowThreshold = Get_lowThreshold();
    int HighThreshold = Get_highThreshold();

    // Apply Canny edge detection
    Debug.Write(wxString::Format("Start detection of solar system object (roi:%d low_tr=%d,high_tr=%d,minr=%d,maxr=%d)\n",
                                 roiActive, LowThreshold, HighThreshold, minRadius, maxRadius));
    Mat edges, dilatedEdges;
    Canny(img8, edges, LowThreshold, HighThreshold, 5, true);
    dilate(edges, dilatedEdges, Mat(), Point(-1, -1), 2);

    // Find contours
    std::vector<std::vector<Point>> contours;
    cv::findContours(dilatedEdges, contours, RETR_LIST, CHAIN_APPROX_NONE);

    // Find total number of contours. If the number is too large, it means that
    // edge detection threshold value is possibly too low, or we'll need to decimate number of points
    // before further processing to avoid performance issues.
    int totalPoints = 0;
    for (const auto& contour : contours)
    {
        totalPoints += contour.size();
    }
    if (totalPoints > 512 * 1024)
    {
        Debug.Write(wxString::Format("Too many contour points detected (%d)\n", totalPoints));
        m_statusMsg = _("Too many contour points detected. Please apply pixel binning, enable ROI, or increase the Edge "
                        "Detection Threshold.");
        pFrame->Alert(m_statusMsg, wxICON_WARNING);
        pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Contour points"), totalPoints);
        return false;
    }

    // Iterate between sets of contours to find the best match
    int contourAllCount = 0;
    int contourMatchingCount = 0;
    float bestScore = 0;
    std::vector<Point2f> bestContour;
    CircleDescriptor bestCircle = { 0 };
    CircleDescriptor bestCentroid = { 0 };
    CircleDescriptor bestDiskCenter = { 0 };
    bestContour.clear();
    int maxThreadsCount = 0;
    for (const auto& contour : contours)
    {
        // Ignore contours with small number of points
        if (contour.size() < 32)
            continue;

        // Find the smallest circle encompassing contour of the object
        // and also center of mass within the contour.
        cv::Moments mu;
        std::vector<Point2f> diskContour;
        CircleDescriptor circle = { 0 };
        CircleDescriptor centroid = { 0 };
        CircleDescriptor diskCenter = { 0 };
        FindCenters(img8, contour, centroid, circle, diskContour, mu, minRadius, maxRadius);

        // Skip circles not within radius range
        if ((circle.radius == 0) || (diskContour.size() == 0))
            continue;

        // Look for a point along the line connecting centers of the smallest circle and center
        // of mass which is equidistant from the outmost edge of the contour. Consider this point as
        // the best match for contour central point.
        CalcLineParams(circle, centroid);
        float score = FindContourCenter(diskCenter, circle, diskContour, mu, minRadius, maxRadius);

        // When user clicks a point in the main window, discard detected features
        // that are far away from it, similar to manual selection of stars in PHD2.
        Point2f circlePoint = { roiRect.x + diskCenter.x, roiRect.y + diskCenter.y };
        if (activeRoiLimits && (norm(clickedPoint - circlePoint) > distanceRoiMax))
            score = 0;

        // Refine the best fit
        if (score > 0.01)
        {
            float searchRadius = 20 * m_eccentricity + 3;
            int threadCount = RefineDiskCenter(score, diskCenter, diskContour, minRadius, maxRadius, searchRadius);
            maxThreadsCount = max(maxThreadsCount, threadCount);
            if (score > bestScore * 0.8)
                threadCount = RefineDiskCenter(score, diskCenter, diskContour, minRadius, maxRadius, 0.5, 0.1);
            maxThreadsCount = max(maxThreadsCount, threadCount);
        }

        // Select best fit based on highest score
        if (score > bestScore)
        {
            bestScore = score;
            bestDiskCenter = diskCenter;
            bestCentroid = centroid;
            bestContour = diskContour;
            bestCircle = circle;
        }
        contourMatchingCount++;
    }

    // Update stats window
    m_planetaryFittingScore = bestScore;
    m_planetaryContourPoints = bestContour.size();
    Debug.Write(wxString::Format(
        "End detection of solar system object (t=%d): r=%.1f, x=%.1f, y=%.1f, score=%.3f, contours=%d/%d, threads=%d\n",
        m_SolarSystemObjWatchdog.Time(), bestDiskCenter.radius, roiRect.x + bestDiskCenter.x, roiRect.y + bestDiskCenter.y,
        bestScore, contourMatchingCount, contourAllCount, maxThreadsCount));
    pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Contours/points"), contourMatchingCount, bestContour.size());
    pFrame->pStatsWin->UpdatePlanetScore(("Fitting score"), bestScore);

    // For use by visual aid for parameter tuning
    if (VisualElementsEnabled())
    {
        m_syncLock.Lock();
        m_roiRect = roiRect;
        m_diskContour = bestContour;
        m_centoid_x = bestCentroid.x;
        m_centoid_y = bestCentroid.y;
        m_sm_circle_x = bestCircle.x;
        m_sm_circle_y = bestCircle.y;
        m_syncLock.Unlock();
    }

    if (bestDiskCenter.radius > 0)
    {
        m_center_x = roiRect.x + bestDiskCenter.x;
        m_center_y = roiRect.y + bestDiskCenter.y;
        m_radius = cvRound(bestDiskCenter.radius);
        m_searchRegion = m_radius;
        return true;
    }

    return false;
}

// Save full 8-bit frame to SER file
void SolarSystemObject::SaveVideoFrame(cv::Mat& FullFrame, cv::Mat& img8, bool roiActive, int bppFactor)
{
    Mat FullFrame8;
    if (roiActive)
        FullFrame.convertTo(FullFrame8, CV_8U, 1.0 / bppFactor);
    else
        FullFrame8 = img8;

    // Create new SER file on first frame or when frame dimensions change - close previous file first
    if (!m_SER || !m_SER->IsOpen() || (m_SER->FrameWidth() != FullFrame.cols) || (m_SER->FrameHeight() != FullFrame.rows))
    {
        // Close previous SER file and open a new one
        if (m_SER && m_SER->IsOpen())
        {
            m_SER->Close();
            delete m_SER;
        }

        // Create new SER file
        wxDateTime dt = wxDateTime::Now();
        const wxString m_serFileName = Debug.GetLogDir() + _("\\") + dt.Format(_T("PHD2_VideoLog_%Y-%m-%d_%H%M%S.ser"));
        m_SER = new SERFile(m_serFileName, FullFrame8.cols, FullFrame8.rows, _("PHD2"), pCamera->Name, pMount->Name());
        m_SER->Open();
    }
    if (m_SER && m_SER->IsOpen())
    {
        m_SER->WriteFrame(FullFrame8);
    }
}

void SolarSystemObject::UpdateDetectionErrorInSimulator(Point2f& clickedPoint)
{
    if (pCamera && pCamera->Name == "Simulator")
    {
        bool errUnknown = true;
        bool clicked = (m_prevClickedPoint != clickedPoint);

        if (m_detected)
        {
            if (m_cameraSimulationRefPointValid)
            {
                m_simulationZeroOffset = false;
                m_cameraSimulationRefPointValid = false;
                m_origPoint = Point2f(m_center_x, m_center_y);
            }
            else if (!m_simulationZeroOffset && !clicked)
            {
                Point2f delta = Point2f(m_center_x, m_center_y) - m_origPoint;
                pFrame->pStatsWin->UpdatePlanetError(_T("Detection error"),
                                                     norm(delta - (m_cameraSimulationMove - m_cameraSimulationRefPoint)));
                errUnknown = false;
            }
        }

        if (errUnknown)
            pFrame->pStatsWin->UpdatePlanetError(_T("Detection error"), -1);

        if (clicked)
        {
            RestartSimulatorErrorDetection();
        }
    }
}

// Get current mount tracking state and rate.
// Returns true if tracking rate is known.
bool SolarSystemObject::GetMountTrackingState(bool& trackingValid, bool& tracking, wxString& rate)
{
    rate = "Sidereal";
    trackingValid = false;
    tracking = false;

    // Cannot get information when no pointing source is available
    if (!pPointingSource || !pPointingSource->IsConnected())
        return false;

    // Get mount tracking state
    if (pPointingSource->GetTracking(&tracking))
        return false;

    // Tracking state is valid
    trackingValid = true;

    // Note: INDI mounts aren't currently supported
    if (pPointingSource->Name().StartsWith(_("INDI Mount")))
        return false;

    enum DriveRates driveRate = driveSidereal;
    double raRate = 0, decRate = 0;
    if (pPointingSource->GetTrackingRate(&driveRate, &raRate, &decRate, false))
        return false;

    // Special handling of EQMOD using RA/DEC offsets from SideReal rate
    switch (driveRate)
    {
    case driveSidereal:
    {
        const double tolerance = 0.00001;
        // Compensate for possible reversal in South hemisphere
        if (raRate > 15.041067)
            raRate -= 15.041067 * 2;
        if ((fabs(raRate - RA_LUNAR_RATE_OFFSET) < tolerance) && (fabs(decRate) < tolerance))
            rate = "Lunar";
        else if ((fabs(raRate - RA_SOLAR_RATE_OFFSET) < tolerance) && (fabs(decRate) < tolerance))
            rate = "Solar";
        else if ((fabs(raRate) > tolerance) || (fabs(decRate) > tolerance))
            rate = "Custom";
        break;
    }
    case driveLunar:
        rate = "Lunar";
        break;
    case driveSolar:
        rate = "Solar";
        break;
    case driveKing:
        rate = "Custom";
        break;
    }

    return true;
}

// Set mount tracking rate
bool SolarSystemObject::SetMountTrackingRate(const wxString& rateStr)
{
    DriveRates trackingRate = driveSidereal;
    wxString rate = rateStr.Lower();
    if (rate == "solar")
        trackingRate = driveSolar;
    else if (rate == "lunar")
        trackingRate = driveLunar;
    else if (rate == "sidereal")
        trackingRate = driveSidereal;
    else if (rate == "stop")
        return !pPointingSource->SetTracking(false);
    else
        return false;

    return !pPointingSource->SetTracking(true) && !pPointingSource->SetTrackingRate(trackingRate);
}

// Check mount tracking state
void SolarSystemObject::CheckMountTrackingState()
{
    // No action when mount is disconnected
    if (!pPointingSource || !pPointingSource->IsConnected())
        return;

    // Get mount tracking state and rate
    wxString rate;
    bool trackingValid, tracking;
    bool rateValid = GetMountTrackingState(trackingValid, tracking, rate);

    // Issue warning alert when mount tracking is disabled during guiding
    if (pFrame->pGuider->IsGuiding())
    {
        const wxString alertMsg = _T("WARNING: mount tracking is disabled!");
        if (trackingValid && !tracking)
        {
            pFrame->Alert(alertMsg, wxICON_WARNING);
            Debug.Write("Find solar system object: tracking is disabled during guiding!\n");
        }
        if (tracking)
            pFrame->ClearAlert(alertMsg);
    }

    if (trackingValid && (tracking != m_prevMountTracking))
    {
        m_prevMountTracking = tracking;
        pFrame->NotifyGuidingParam("Mount Tracking", tracking);
    }

    // Notify clients about changes in tracking rate
    if (rateValid && (rate != m_prevTrackingRate))
    {
        m_prevTrackingRate = rate;
        pFrame->NotifyGuidingParam("Tracking Rate", rate);
    }
}

// Find object in the given image
bool SolarSystemObject::FindSolarSystemObject(const usImage *pImage, bool autoSelect)
{
    m_SolarSystemObjWatchdog.Start();

    // Default error status message
    m_statusMsg = _("Object not found");

    // Check tracking state
    CheckMountTrackingState();

    // Skip detection when paused
    if (m_paramDetectionPaused)
    {
        m_syncLock.Lock();
        m_detected = false;
        m_detectionCounter = 0;
        m_diskContour.clear();
        m_surf.inlierPoints.clear();
        m_syncLock.Unlock();
        return false;
    }

    // Auto select star was requested
    if (autoSelect)
    {
        m_clicked_x = 0;
        m_clicked_y = 0;
        m_userLClick = false;
        m_detectionCounter = 0;
        RestartSimulatorErrorDetection();
    }
    Point2f clickedPoint = Point2f(m_clicked_x, m_clicked_y);

    // Use ROI for CPU time optimization
    bool roiActive = false;
    int minRadius = (int) Get_minRadius();
    int maxRadius = (int) Get_maxRadius();
    int roiRadius = (int) (maxRadius * 3 / 2.0 + 0.5);
    int roiOffsetX = 0;
    int roiOffsetY = 0;
    Mat FullFrame(pImage->Size.GetHeight(), pImage->Size.GetWidth(), CV_16UC1, pImage->ImageData);

    // Refuse to process images larger than 4096x4096 and request to use camera binning
    if (FullFrame.cols > 4096 || FullFrame.rows > 4096)
    {
        Debug.Write(wxString::Format("Find solar system object: image is too large %dx%d\n", FullFrame.cols, FullFrame.rows));
        pFrame->Alert(_("ERROR: camera frame size exceeds maximum limit. Please apply binning to reduce the frame size."),
                      wxICON_ERROR);
        m_syncLock.Lock();
        m_detected = false;
        m_detectionCounter = 0;
        m_diskContour.clear();
        m_surf.inlierPoints.clear();
        m_syncLock.Unlock();
        return false;
    }

    // Limit image processing to ROI when enabled
    Mat RoiFrame;
    Rect roiRect(0, 0, pImage->Size.GetWidth(), pImage->Size.GetHeight());
    if (!autoSelect && GetRoiEnableState() && m_detected && !GetSurfaceTrackingState() && (m_center_x < m_frameWidth) &&
        (m_center_y < m_frameHeight) && (m_frameWidth == pImage->Size.GetWidth()) &&
        (m_frameHeight == pImage->Size.GetHeight()))
    {
        float fraction = (m_userLClick && (m_detectionCounter <= 4)) ? (1.0 - m_detectionCounter / 4.0) : 0.0;
        int x = cvRound(m_clicked_x * fraction + m_center_x * (1.0 - fraction));
        int y = cvRound(m_clicked_y * fraction + m_center_y * (1.0 - fraction));
        roiOffsetX = wxMax(0, x - roiRadius);
        roiOffsetY = wxMax(0, y - roiRadius);
        int w = wxMin(roiRadius * 2, pImage->Size.GetWidth() - roiOffsetX);
        int h = wxMin(roiRadius * 2, pImage->Size.GetHeight() - roiOffsetY);
        roiRect = Rect(roiOffsetX, roiOffsetY, w, h);
        RoiFrame = FullFrame(roiRect);
        roiActive = true;
    }
    else
    {
        RoiFrame = FullFrame;
    }

    // Make sure to use 8-bit gray image for feature detection
    // pImage always has 16-bit pixels, but depending on camera bpp
    // we should properly scale the image.
    Mat img8;
    int bppFactor = (pImage->BitsPerPixel >= 8) ? 1 << (pImage->BitsPerPixel - 8) : 1;
    RoiFrame.convertTo(img8, CV_8U, 1.0 / bppFactor);

    // Save latest frame dimensions
    m_frameWidth = pImage->Size.GetWidth();
    m_frameHeight = pImage->Size.GetHeight();

    // Save frames to SER file only when guiding is active
    if (GetVideoLogging() && pFrame->pGuider->IsGuiding())
    {
        SaveVideoFrame(FullFrame, img8, roiActive, bppFactor);
    }

    // ROI current state and limit
    bool activeRoiLimits = m_userLClick && GetRoiEnableState();
    float distanceRoiMax = maxRadius * 3 / 2.0;

    bool detectionResult = false;
    try
    {
        // Do slight image blurring to decrease noise impact on results
        Mat imgFiltered;
        GaussianBlur(img8, imgFiltered, cv::Size(3, 3), 1.5);

#ifdef DEVELOPER_MODE
        // Optional noise suppression filter
        if (GetNoiseFilterState())
        {
            const int d = 10;
            const double sigmaColor = 5.0;
            double sigmaSpace = 5.0;
            Mat filteredImage;
            bilateralFilter(imgFiltered, filteredImage, d, sigmaColor, sigmaSpace);
            imgFiltered = filteredImage;
            Debug.Write(_("Find solar system object: noise filter applied\n"));
        }
#endif

        // Find object depending on the selected detection mode
        switch (GetPlanetDetectMode())
        {
        case DETECTION_MODE_SURFACE:
            detectionResult = DetectSurfaceFeatures(imgFiltered, clickedPoint, autoSelect);
            pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Features"), detectionResult ? m_detectedFeatures : 0);
            break;
        case DETECTION_MODE_DISK:
            detectionResult = FindOrbisCenter(imgFiltered, minRadius, maxRadius, roiActive, clickedPoint, roiRect,
                                              activeRoiLimits, distanceRoiMax);
            break;
        }

        // Calculate sharpness of the image
        if (m_measuringSharpnessMode)
        {
            m_focusSharpness = CalcSharpness(FullFrame, clickedPoint, detectionResult);
            Debug.Write(wxString::Format("Find solar system object: sharpness=%.1f\n", m_focusSharpness));
        }

        // Update detection time stats
        pFrame->pStatsWin->UpdatePlanetDetectionTime(m_SolarSystemObjWatchdog.Time());

        if (detectionResult)
        {
            m_detected = true;
            if (m_detectionCounter++ > 3)
            {
                // Smooth search region to avoid sudden jumps in star find stats
                m_searchRegion = cvRound(m_searchRegion * 0.3 + m_prevSearchRegion * 0.7);

                // Forget about the clicked point after a few successful detections
                m_userLClick = false;
            }
            m_prevSearchRegion = m_searchRegion;
        }
        if (m_measuringSharpnessMode || detectionResult)
            m_unknownHFD = false;
    }
    catch (const wxString& msg)
    {
        POSSIBLY_UNUSED(msg);
        Debug.Write(wxString::Format("Find solar system object: exception %s\n", msg));
    }
    catch (const cv::Exception& ex)
    {
        // Handle OpenCV exceptions
        Debug.Write(wxString::Format("Find solar system object: OpenCV exception %s\n", ex.what()));
        pFrame->Alert(_("ERROR: exception occurred during image processing: change detection parameters"), wxICON_ERROR);
    }
    catch (...)
    {
        // Handle any other exceptions
        Debug.Write("Find solar system object: unknown exception\n");
        pFrame->Alert(_("ERROR: unknown exception occurred in solar system object detection"), wxICON_ERROR);
    }

    // For simulated camera, calculate detection error by comparing with the simulated position
    UpdateDetectionErrorInSimulator(clickedPoint);

    // Update data shared with other thread
    m_syncLock.Lock();
    m_roiRect = roiRect;
    if (!detectionResult)
    {
        m_detected = false;
        m_detectionCounter = 0;
        m_diskContour.clear();
        m_surf.inlierPoints.clear();
    }
    m_roiActive = roiActive;
    m_prevClickedPoint = clickedPoint;
    m_syncLock.Unlock();

    return detectionResult;
}