/*
 *  planetary.cpp
 *  PHD Guiding
 *
 *  Solar, lunar and planetary detection extensions by Leo Shatz
 *  Copyright (c) 2023-2026 Leo Shatz
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
#include "cam_FrameMon.h"
#include "frame_export.h"
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
SolarSystemObject::SolarSystemObject() : m_remoteDetCond(m_remoteDetMutex)
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
    m_updateDiameters = false;
    m_disableCustomRateOverride = false;
    m_remoteData = false;
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
    float sigma = 2.0;
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
    SetSurfaceTrackingState(false);
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

    // Save PHD2 settings we change for solar system object guiding
    m_phd2_MassChangeThresholdEnabled = pConfig->Profile.GetBoolean("/guider/onestar/MassChangeThresholdEnabled", false);
    m_phd2_UseSubframes = pConfig->Profile.GetBoolean("/camera/UseSubframes", false);
    m_phd2_MultistarEnabled = pConfig->Profile.GetBoolean("/guider/multistar/enabled", true);

    // Remove the alert dialog setting for pausing solar/planetary detection
    pConfig->Global.DeleteEntry(PausePlanetDetectionAlertEnabledKey());
}

SolarSystemObject::~SolarSystemObject()
{
    CloseVideoLog();

    // Save all detection parameters
#ifdef DEVELOPER_MODE
    pConfig->Profile.SetBoolean("/PlanetTool/noise_filter", GetNoiseFilterState());
#endif
    pConfig->Profile.SetInt("/PlanetTool/min_radius", Get_minRadius());
    pConfig->Profile.SetInt("/PlanetTool/max_radius", Get_maxRadius());
    pConfig->Profile.SetInt("/PlanetTool/high_threshold", Get_highThreshold());
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

// Enable/disable surface tracking mode
void SolarSystemObject::Set_SurfaceDetectionMode(bool enabled)
{
    // Don't change the state if it's already set
    if (enabled == m_paramSurfaceTracking)
        return;

    // Request to set the new state
    SetSurfaceTrackingState(enabled);
}

// Set limits for min/max radius
bool SolarSystemObject::SetLimits(int minRadius, int maxRadius)
{
    if (minRadius > 0 && maxRadius > 0)
    {
        Set_minRadius(minRadius);
        Set_maxRadius(maxRadius);
        SetMinMaxDiametersUpdate();
        return true;
    }
    return false;
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

double SolarSystemObject::CalcSharpness(const cv::Mat& src, int filtMin, int filtMax)
{
    cv::Mat filtered, normf;
    cv::Mat norm = cv::Mat(src.size(), CV_16U);
    Mat grad_x, grad_y, grad;
    GaussianBlur(src, filtered, cv::Size(3, 3), 1.6);
    const int range = std::max(filtMax - filtMin, 1);
    const float scale = 65535.0f / range;
    for (int y = 0; y < src.rows; y++)
    {
        for (int x = 0; x < src.cols; x++)
        {
            unsigned pixel = filtered.at<ushort>(y, x);
            unsigned normPixel = (pixel < filtMin) ? 0 : (pixel > filtMax ? 0xffff : (pixel - filtMin) * scale);
            norm.at<ushort>(y, x) = normPixel;
        }
    }
    norm.convertTo(normf, CV_32F);
    Sobel(normf, grad_x, CV_32F, 1, 0, 3);
    Sobel(normf, grad_y, CV_32F, 0, 1, 3);
    magnitude(grad_x, grad_y, grad);
    return cv::mean(grad)[0] / 100.0;
}

static double estimateSigma(const cv::Mat& image)
{
    cv::Mat flt, lap;
    if (image.type() != CV_32F)
    {
        image.convertTo(flt, CV_32F);
        cv::Laplacian(flt, lap, CV_32F);
    }
    else
    {
        cv::Laplacian(image, lap, CV_32F);
    }
    cv::Mat absLap = cv::abs(lap);

    double minVal, maxVal;
    cv::minMaxLoc(absLap, &minVal, &maxVal);
    if (maxVal - minVal < 0.001)
        return 0;

    std::vector<int> vec;
    vec.reserve(absLap.rows * absLap.cols);
    double factor = 65535.0 / (maxVal - minVal);
    for (int y = 0; y < absLap.rows; y++)
    {
        for (int x = 0; x < absLap.cols; x++)
        {
            float val = absLap.at<float>(y, x);
            if (val > minVal)
            {
                vec.push_back(cvRound((val - minVal) * factor));
            }
        }
    }
    size_t n = vec.size() / 2;
    std::nth_element(vec.begin(), vec.begin() + n, vec.end());
    return (vec[n] + minVal) / factor / 0.6745;
}

void SolarSystemObject::CalcPlanetMetrics(const cv::Mat& image, int center_x, int center_y, int r, int annulusWidth, double& mass,
                                          double& snr, int& peak)
{
    const double sigma_factor = 3.0;
    int scopeOuter = r + annulusWidth * 3;
    int scopeInner = r + annulusWidth;
    int scopeOuter2 = scopeOuter * scopeOuter;
    int scopeInner2 = scopeInner * scopeInner;
    int start_x = std::max(0, center_x - scopeOuter);
    int end_x = std::min(center_x + scopeOuter, image.cols - 1);
    int start_y = std::max(0, center_y - scopeOuter);
    int end_y = std::min(center_y + scopeOuter, image.rows - 1);

    // Calculate the statistics within the larger annulus
    double sum = 0.0;
    double sq_sum = 0.0;
    int count = 0;
    for (int y = start_y; y <= end_y; y++)
    {
        for (int x = start_x; x <= end_x; x++)
        {
            int r2 = (x - center_x) * (x - center_x) + (y - center_y) * (y - center_y);
            if ((r2 < scopeOuter2) && (r2 > scopeInner2))
            {
                unsigned int pixel = image.at<ushort>(y, x);
                sum += pixel;
                sq_sum += pixel * pixel;
                count++;
            }
        }
    }

    // Calculate mean, variance and standard deviation in the annulus
    double mean = (count > 0) ? sum / count : 0.0;
    double variance = (count > 0) ? (sq_sum / count) - (mean * mean) : 0.0;
    double stdDev = sqrt(variance);
    unsigned int signalThreshold = (unsigned int) (mean + stdDev * sigma_factor);

    // Calculate signal and noise within the circle
    cv::Mat inner = cv::Mat::zeros(image.size(), CV_16U);
    unsigned int peak_val = 0;
    double meanSignal = 0.0;
    int signalCount = 0;
    double meanNoise = 0.0;
    int noiseCount = 0;
    double noiseVariance = 0.0;
    for (int y = start_y; y <= end_y; y++)
    {
        for (int x = start_x; x <= end_x; x++)
        {
            int r2 = (x - center_x) * (x - center_x) + (y - center_y) * (y - center_y);
            unsigned int pixel = image.at<ushort>(y, x);
            inner.at<ushort>(y, x) = pixel;
            if (pixel > peak_val)
                peak_val = pixel;
            if (r2 < scopeInner2)
            {
                if (pixel > signalThreshold)
                {
                    meanSignal += pixel;
                    signalCount++;
                }
            }
            else if (r2 < scopeOuter2)
            {
                unsigned int pixel = image.at<ushort>(y, x);
                if (pixel <= signalThreshold)
                {
                    meanNoise += pixel;
                    noiseVariance += pixel * pixel;
                    noiseCount++;
                }
            }
        }
    }

    // Estimate standard noise deviation based on MAD in the inner circle
    float sigma = (float) estimateSigma(inner);

    // Estimate mass as the sum of all pixels normalized by the circle area.
    mass = (float) meanSignal / (float) (CV_PI * r * r);
    meanSignal = (signalCount > 0) ? meanSignal / signalCount : 0.0;

    snr = 100.0f;
    float noiseStdDev = 0.0f;
    if (noiseCount > 0)
    {
        meanNoise /= noiseCount;
        noiseVariance = noiseVariance / noiseCount - meanNoise * meanNoise;
        noiseStdDev = (float) sqrt(noiseVariance);
    }

    // Use the maximum of noise sigma estimations
    noiseStdDev = std::max(noiseStdDev, sigma);
    if (noiseStdDev > 1)
    {
        snr = (meanSignal - meanNoise) / noiseStdDev;
        snr = std::max(0.001, snr);
        snr = 20.0f * log10(snr);
    }
    m_noiseStdDev = noiseStdDev;

    peak = peak_val;
    Debug.Write(wxString::Format(
        "CalcPlanetMetrics: signalThreshold=%.1f, meanSignal=%.1f, meanNoise=%.1f (sigma=%.1f), SNR=%.1f\n",
        signalThreshold, meanSignal, meanNoise, sigma, m_snr));
}

// Get planetary metrics
void SolarSystemObject::GetPlanetMetrics(double& snr, double& mass, double& hfd, unsigned short& peak)
{
    snr = m_snr;
    mass = m_mass;
    hfd = GetHFD();
    peak = m_peak;
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

// Notification callback when user clicks on the image
void SolarSystemObject::OnLClick(usImage *pImage, double& x, double& y)
{
    if (pFrame->GetStarFindMode() == Star::FIND_PLANET)
    {
        m_clicked_x = wxMin(x, pImage->Size.GetWidth() - 1);
        m_clicked_y = wxMin(y, pImage->Size.GetHeight() - 1);
        m_userLClick = true;
        m_detectionCounter = 0;
#if defined(FRAME_MONITOR_CAMERA)
        if (pCamera && pCamera->Connected)
        {
            if (m_detected && !m_paramSurfaceTracking)
            {
                x = m_center_x;
                y = m_center_y;
            }
            EvtServer.NotifyMouseClick(PHD_Point(x, y));
        }
#endif
    }
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
    wxThread::ExitCode Entry()
    {
        SetThreadName("PHD2 Planetary Score");
        for (const Point2f& point : points)
        {
            float pointRadius = 0;
            float score = ::CalcContourScore(pointRadius, point, contour, minRadius, maxRadius);
            if (score > threadBestScore)
            {
                threadBestScore = score;
                radius = pointRadius;
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
void SolarSystemObject::FindCenters(const Mat& image, const std::vector<Point>& contour, CircleDescriptor& centroid,
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

// Get surface features
bool SolarSystemObject::GetSurfaceFeatures()
{
    // No detected features
    m_remoteData = false;
    m_detected = false;
    m_detectedFeatures = 0;
    m_focusSharpness = 0;
    m_peak = 0;
    m_searchRegion = 128;

    // Check if tracking info is available
    if (pCamera && pCamera->Connected)
    {
        frameDesc desc;
        bool bErr = pCamera->GetCaptureDescriptor(&desc);
        if (!bErr)
        {
            m_detectionTime = desc.time;
            m_snr = desc.snr;
            m_mass = desc.mass;
            m_peak = desc.peak;
            m_focusSharpness = desc.sharpness;
            m_surf.variance = desc.dispersion;
            m_surf.trackingQuality = desc.quality;
            m_detectedFeatures = desc.features;
            m_radius = m_starProfileSize;
            if (desc.pos.IsValid())
            {
                m_center_x = desc.pos.X;
                m_center_y = desc.pos.Y;
                m_detected = true;
            }
            m_remoteData = true;
            pFrame->pStatsWin->UpdatePlanetScore(_T("Dispersion"), m_surf.variance);
            pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Features"), m_detected ? m_detectedFeatures : 0);
            return m_detected;
        }
    }

    return false;
}

// Find orb center using circle matching with contours
bool SolarSystemObject::FindOrbisCenter(const Mat& src, int minRadius, int maxRadius, bool roiActive, Point2f& clickedPoint,
                                        Rect& roiRect, bool activeRoiLimits, float distanceRoiMax)
{
    m_planetaryContourPoints = 0;
    m_planetaryFittingScore = 0;
    m_remoteData = false;
    m_focusSharpness = 0;
    m_peak = 0;

    // Check if tracking info is available
    if (pCamera && pCamera->Connected)
    {
        frameDesc desc;
        bool bErr = pCamera->GetCaptureDescriptor(&desc);
        if (!bErr)
        {
            m_detectionTime = desc.time;
            m_planetaryContourPoints = desc.features;
            m_planetaryFittingScore = desc.quality;
            m_detected = false;
            m_focusSharpness = desc.sharpness;
            m_snr = desc.snr;
            m_mass = desc.mass;
            m_peak = desc.peak;
            if (desc.pos.IsValid())
            {
                m_detectedFeatures = desc.features;
                m_radius = desc.radius;
                SetLimits(desc.minRadius, desc.maxRadius);
                m_center_x = desc.pos.X;
                m_center_y = desc.pos.Y;
                m_detected = true;
            }
            pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Contour points"), m_planetaryContourPoints);
            pFrame->pStatsWin->UpdatePlanetScore(("Fitting score"), m_planetaryFittingScore);
            m_remoteData = true;

            return m_detected;
        }
    }

    // Do slight image blurring to decrease noise impact on results
    Mat img8;
    GaussianBlur(src, img8, cv::Size(3, 3), 1.5);

    // Apply Canny edge detection
    int LowThreshold = Get_lowThreshold();
    int HighThreshold = Get_highThreshold();
    Debug.Write(wxString::Format("Start disk detection: roi:%d, minr=%d,maxr=%d (low_tr=%d,high_tr=%d)\n",
        roiActive, minRadius, maxRadius, LowThreshold, HighThreshold));
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

void SolarSystemObject::SetVideoLogging(bool enable)
{
    m_videoLogEnabled = enable;
    if (!enable)
        CloseVideoLog();
}

void SolarSystemObject::CloseVideoLog()
{
    if (m_SER)
    {
        if (m_SER->IsOpen())
            m_SER->Close();
        delete m_SER;
        m_SER = nullptr;
    }
}

// Save full 8-bit frame to SER file
void SolarSystemObject::SaveVideoFrame(const cv::Mat& src, int bppFactor)
{
    Mat frame8;
    src.convertTo(frame8, CV_8U, 1.0 / bppFactor);

    // Create new SER file on first frame or when frame dimensions change - close previous file first
    if (!m_SER || !m_SER->IsOpen() || (m_SER->FrameWidth() != src.cols) || (m_SER->FrameHeight() != src.rows))
    {
        // Close previous SER file and open a new one
        CloseVideoLog();

        // Create new SER file
        wxDateTime dt = wxDateTime::Now();
        const wxString m_serFileName = Debug.GetLogDir() + _("\\") + dt.Format(_T("PHD2_VideoLog_%Y-%m-%d_%H%M%S.ser"));
        m_SER = new SERFile(m_serFileName, src.cols, src.rows, _("PHD2"), pCamera->Name, pMount->Name());
        if (!m_SER->Open())
            CloseVideoLog();
    }
    if (m_SER && m_SER->IsOpen())
    {
        m_SER->WriteFrame(frame8);
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
bool SolarSystemObject::GetMountTrackingState(bool& trackingValid, bool& tracking, wxString& rate,
    bool& offsetsValid, double& raOffset, double& decOffset)
{
    rate = "Sidereal";
    trackingValid = false;
    offsetsValid = false;
    tracking = false;

    // Cannot get information when no pointing source is available
    if (!pPointingSource || !pPointingSource->IsConnected())
        return false;

    // Get mount tracking state
    if (pPointingSource->GetTracking(&tracking))
        return false;

    // Note: INDI mounts aren't currently supported
    if (pPointingSource->Name().StartsWith(_("INDI Mount")))
        return false;

    enum DriveRates driveRate = driveSidereal;
    double raRate = 0, decRate = 0;
    if (pPointingSource->GetTrackingRate(&driveRate, &raRate, &decRate, false))
        return false;

    // Get RA/DEC offsets from SideReal rate
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
        rate = "King";
        break;
    }

    // Tracking state is valid
    trackingValid = true;
    offsetsValid = true;
    raOffset = raRate;
    decOffset = decRate;
    return true;
}

// Set mount tracking rate
bool SolarSystemObject::SetMountTrackingRate(const wxString& rateStr, double ra_offset, double dec_offset)
{
    if (!pPointingSource)
    {
        Debug.Write(wxString::Format("Failed to set tracking rate %s: no pointing source\n", rateStr));
        return false;
    }

    SetDisableCustomRateOverride();

    DriveRates trackingRate = driveSidereal;
    wxString rate = rateStr.Lower();
    if (rate == "solar")
        trackingRate = driveSolar;
    else if (rate == "lunar")
        trackingRate = driveLunar;
    else if (rate == "sidereal")
        trackingRate = driveSidereal;
    else if (rate == "custom") {
        if (pPointingSource->SetTrackingRate(driveSidereal))
            return false;
        double const siderealSecsPerSec = 0.9973;
        const double siderealRate = 3600.0 / (15.0 * siderealSecsPerSec);
        if ((fabs(ra_offset) > 10 * siderealRate) || (fabs(dec_offset) > 10 * siderealRate))
            return false;
        bool bErr = pPointingSource->SetTrackingRateOffsets(ra_offset, dec_offset);
        if (bErr)
            Debug.Write(wxString::Format("%s custom tracking rate: %.6f, %.6f\n", bErr ? "Failed to set" : "Set", ra_offset, dec_offset));
        return !bErr;

    }
    else if (rate == "start")
        return !pPointingSource->SetTracking(true);
    else if (rate == "stop")
        return !pPointingSource->SetTracking(false);
    else
        return false;

    pPointingSource->SetTrackingRateOffsets(0, 0);
    bool bErr = pPointingSource->SetTrackingRate(trackingRate);
    if (bErr)
        Debug.Write(wxString::Format("Failed to set tracking rate: %s\n", rate));
    else
        Debug.Write(wxString::Format("Set custom tracking rate: %s\n", rate));
    return !bErr;
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
    bool offsetsValid;    
    double raOffset, decOffset;
    bool rateValid = GetMountTrackingState(trackingValid, tracking, rate, offsetsValid, raOffset, decOffset);

    // No tracking-disabled alert here - the parked/tracking gate in Mount::MoveOffset will
    // surface "Guiding stopped: the mount is not tracking." via OnMoveComplete on the next
    // pulse, and Scope::SetLastKnownTracking auto-clears it when tracking resumes.
    if (trackingValid)
        Debug.Write(wxString::Format("CheckMountTrackingState: tracking=%d\n", tracking));

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
#define REMOTE_DETECTION_TIMEOUT_MS 15000

void SolarSystemObject::SetRemoteDetectionResult(const RemoteDetection& result)
{
    wxMutexLocker lock(m_remoteDetMutex);
    m_remoteDetection = result;
    m_remoteDetection.valid = true;
    m_remoteDetCond.Broadcast();
}

void SolarSystemObject::WaitRemoteDetection(uint32_t frame)
{
    EvtServer.NotifyDetectionRequest((int) frame);

    wxMutexLocker lock(m_remoteDetMutex);
    wxStopWatch sw;
    while (!(m_remoteDetection.valid && m_remoteDetection.frame >= frame))
    {
        long remaining = REMOTE_DETECTION_TIMEOUT_MS - sw.Time();
        if (remaining <= 0 || !FrameExport::IsEnabled())
            break;
        m_remoteDetCond.WaitTimeout(remaining < 200 ? remaining : 200);
    }
}

bool SolarSystemObject::ConsumeRemoteDetection()
{
    uint32_t frame = FrameExport::CurrentFrame();
    RemoteDetection result;
    {
        wxMutexLocker lock(m_remoteDetMutex);
        result = m_remoteDetection;
    }

    m_remoteData = true;
    m_detectionTime = -1;

    if (!result.valid || result.frame < frame || !result.detected)
    {
        m_detected = false;
        return false;
    }

    m_center_x = result.x;
    m_center_y = result.y;
    m_radius = result.radius;
    m_searchRegion = result.radius;
    SetLimits(result.minRadius, result.maxRadius);
    m_snr = result.snr;
    m_mass = result.mass;
    m_peak = result.peak;
    m_focusSharpness = result.sharpness;
    m_detectedFeatures = result.features;
    m_planetaryContourPoints = result.features;
    m_planetaryFittingScore = result.quality;
    m_surf.variance = result.dispersion;
    if (result.roiW > 0 && result.roiH > 0)
    {
        m_roiRect = Rect(result.roiX, result.roiY, result.roiW, result.roiH);
        m_roiActive = true;
    }
    m_detected = true;

    pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Contour points"), m_planetaryContourPoints);
    pFrame->pStatsWin->UpdatePlanetScore(("Fitting score"), m_planetaryFittingScore);
    return true;
}

bool SolarSystemObject::FindSolarSystemObject(const usImage *pImage, bool autoSelect)
{
    m_SolarSystemObjWatchdog.Start();

    // Default error status message
    m_statusMsg = _("Object not found");

    // Check tracking state
    CheckMountTrackingState();

#if defined(FRAME_MONITOR_CAMERA)
    // Notify client of autoselect attempt. Sent for any connected camera, not just the FrameMon
    // camera, so HM is also notified when driving detection over the shared-memory path with a real
    // camera (mirrors the NotifyMouseClick handling in OnLClick).
    if (autoSelect && pCamera && pCamera->Connected)
        EvtServer.NotifyAutoSelect();
#endif

    // Skip detection when paused
    if (m_paramDetectionPaused)
    {
        m_syncLock.Lock();
        m_detected = false;
        m_detectionCounter = 0;
        m_diskContour.clear();
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
        m_syncLock.Unlock();
        return false;
    }

    // In HM-driven mode detection is offloaded, so the ROI crop and 8-bit
    // conversion below are skipped to keep the real-time path slim.
#if defined(FRAME_MONITOR_CAMERA)
    bool hmDriven = FrameExport::IsEnabled() && pCamera && pCamera->Name != FRAME_MONITOR_CAMERA;
#else
    bool hmDriven = false;
#endif

    // Limit image processing to ROI when enabled
    Mat RoiFrame;
    Rect roiRect(0, 0, pImage->Size.GetWidth(), pImage->Size.GetHeight());
    if (!hmDriven && !autoSelect && GetRoiEnableState() && m_detected && !GetSurfaceTrackingState() &&
        (m_center_x < m_frameWidth) && (m_center_y < m_frameHeight) && (m_frameWidth == pImage->Size.GetWidth()) &&
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
    if (!hmDriven)
        RoiFrame.convertTo(img8, CV_8U, 1.0 / bppFactor);

    // Save latest frame dimensions
    m_frameWidth = pImage->Size.GetWidth();
    m_frameHeight = pImage->Size.GetHeight();
    m_detectionTime = -1;

    // Save frames to SER file only when guiding is active
    if (GetVideoLogging() && pFrame->pGuider->IsGuiding())
    {
        SaveVideoFrame(FullFrame, bppFactor);
    }
    else if (m_SER && m_SER->IsOpen())
    {
        CloseVideoLog();
    }

    // ROI current state and limit
    bool activeRoiLimits = m_userLClick && GetRoiEnableState();
    float distanceRoiMax = maxRadius * 3 / 2.0;

    bool detectionResult = false;
    try
    {
        if (hmDriven)
            detectionResult = ConsumeRemoteDetection();
        else
            // Find object depending on the selected detection mode
            switch (GetPlanetDetectMode())
            {
            case DETECTION_MODE_SURFACE:
                detectionResult = GetSurfaceFeatures();
                break;
            case DETECTION_MODE_DISK:
                detectionResult = FindOrbisCenter(img8, minRadius, maxRadius, roiActive, clickedPoint, roiRect,
                                                  activeRoiLimits, distanceRoiMax);
                break;
            }
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

    try
    {
        // Compute image data metrics (snr, peak and mass)
        if (!m_remoteData && detectionResult && (GetPlanetDetectMode() == DETECTION_MODE_DISK))
            CalcPlanetMetrics(FullFrame, m_center_x, m_center_y, m_radius, 25, m_mass, m_snr, m_peak);

        // Calculate sharpness of the image regardless of detection
        if (m_measuringSharpnessMode && (m_focusSharpness == 0))
            m_focusSharpness = CalcSharpness(FullFrame, pImage->FiltMin, pImage->FiltMax);
        Debug.Write(wxString::Format("Find solar system object: sharpness=%.1f\n", m_focusSharpness));
    }
    catch (const cv::Exception& ex)
    {
        Debug.Write(wxString::Format("Find solar system object: OpenCV exception %s\n", ex.what()));
    }

    // Frame latency: from end of frame acquisition until detection results are ready
    double frameLatencyMs = (pImage->AcqEndUs > 0) ? (SteadyClockUs() - pImage->AcqEndUs) / 1000.0 : -1.0;
    pFrame->pStatsWin->UpdateFrameLatency(frameLatencyMs);

    // Notify the server about the detection result
    if (!m_remoteData && GetPlanetDetectMode() == DETECTION_MODE_DISK)
    {
        EvtServer.NotifyPlanetaryDetection(detectionResult, m_planetaryContourPoints, m_planetaryFittingScore, m_radius);
    }

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
    }
    m_roiActive = roiActive;
    m_prevClickedPoint = clickedPoint;
    m_syncLock.Unlock();

    return detectionResult;
}
