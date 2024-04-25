/*
 *  guider_planetary.cpp
 *  PHD Guiding
 *
 *  Planetary detection extensions by Leo Shatz
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
#include "guider_planetary.h"
#include "planetary_tool.h"

#include <algorithm>
#include <numeric>

#if ((wxMAJOR_VERSION < 3) && (wxMINOR_VERSION < 9))
#define wxPENSTYLE_DOT wxDOT
#endif

// Using OpenCV namespace
using namespace cv;

// Gaussian weights lookup table
static float gaussianWeight[2000];

// Initialize planetary module
GuiderPlanet::GuiderPlanet()
{
    m_Planetary_enabled = false;
    m_PlanetaryDetectionPaused = false;
    m_Planetary_SurfaceTracking = false;
    m_prevCaptureActive = false;
    m_detected = false;
    m_radius = 0;
    m_searchRegion = 0;
    m_prevSearchRegion = 0;
    m_referenceKeypoints.clear();
    m_inlierPoints.clear();
    m_surfaceDetectionParamsChanging = false;
    m_trackingQuality = 0;
    m_starProfileSize = 50;
    m_measuringSharpnessMode = false;
    m_unknownHFD = true;
    m_focusSharpness = 0;
    m_Planetary_NoiseFilterState = false;

    m_guidingFixationPointValid = false;
    m_surfaceFixationPoint = Point2f(0, 0);
    m_guidingFixationPoint = Point2f(0, 0);
    m_cameraSimulationMove = Point2f(0, 0);
    m_cameraSimulationRefPoint = Point2f(0, 0);
    m_cameraSimulationRefPointValid = false;
    m_simulationZeroOffset = false;
    m_center_x = m_center_y = 0;
    m_origPoint = Point2f(0, 0);

    m_cachedScaledWidth = 0;
    m_cachedScaledHeight = 0;
    m_cachedTrackerImage = NULL;

    m_roiClicked = false;
    m_roiActive = false;
    m_detectionCounter = 0;
    m_clicked_x = 0;
    m_clicked_y = 0;
    m_prevClickedPoint = Point2f(0, 0);
    m_diskContour.clear();
    m_RoiEnabled = false;
    m_Planetary_minRadius = PT_MIN_RADIUS_DEFAULT;
    m_Planetary_maxRadius = PT_MAX_RADIUS_DEFAULT;
    m_Planetary_lowThreshold = PT_HIGH_THRESHOLD_DEFAULT / 2;
    m_Planetary_highThreshold = PT_HIGH_THRESHOLD_DEFAULT;
    m_Planetary_minHessian = PT_MIN_HESSIAN_UI_DEFAULT;
    m_Planetary_maxFeatures = PT_MAX_SURFACE_FEATURES;
    m_Planetary_ShowElementsButtonState = false;
    m_Planetary_ShowElementsVisual = false;
    m_draw_PlanetaryHelper = false;
    m_frameWidth = 0;
    m_frameHeight = 0;
    m_PlanetEccentricity = 0;
    m_PlanetAngle = 0;
    m_trackedFeatureSize = TRACKING_FEATURE_SIZE_UNDEF;

    m_videoLogEnabled = false;
    m_SER = nullptr;

    // Build gaussian weighting function table used for circle feature detection
    float sigma = 1.0;
    memset(gaussianWeight, 0, sizeof(gaussianWeight));
    for (double x = 0; x < 20; x += 0.01)
    {
        int i = x * 100 + 0.5;
        if (i < ARRAYSIZE(gaussianWeight))
            gaussianWeight[i] += exp(-(pow(x, 2) / (2 * pow(sigma, 2))));
    }

    // Load lock target bitmap
#   include "icons/target_lock_ok.png.h"
    wxBitmap lockTargetBitmapOk(wxBITMAP_PNG_FROM_DATA(target_lock_ok));
    m_lockTargetImageOk = wxImage(lockTargetBitmapOk.ConvertToImage());

    // Extract width and height from IHDR chunk data (9-12 bytes for width, 13-16 bytes for height)
    const uchar* ihdrChunk1 = target_lock_ok_png + 16;
    m_lockTargetWidthOk = (ihdrChunk1[0] << 24) | (ihdrChunk1[1] << 16) | (ihdrChunk1[2] << 8) | ihdrChunk1[3];
    m_lockTargetHeightOk = (ihdrChunk1[4] << 24) | (ihdrChunk1[5] << 16) | (ihdrChunk1[6] << 8) | ihdrChunk1[7];

#   include "icons/target_lock_bad.png.h"
    wxBitmap lockTargetBitmapBad(wxBITMAP_PNG_FROM_DATA(target_lock_bad));
    m_lockTargetImageBad = wxImage(lockTargetBitmapBad.ConvertToImage());

    // Extract width and height from IHDR chunk data (9-12 bytes for width, 13-16 bytes for height)
    const uchar* ihdrChunk2 = target_lock_ok_png + 16;
    m_lockTargetWidthBad = (ihdrChunk2[0] << 24) | (ihdrChunk2[1] << 16) | (ihdrChunk2[2] << 8) | ihdrChunk2[3];
    m_lockTargetHeightBad = (ihdrChunk2[4] << 24) | (ihdrChunk2[5] << 16) | (ihdrChunk2[6] << 8) | ihdrChunk2[7];

    // Get initial values of the planetary tracking state and parameters from configuration
    SetSurfaceTrackingState(pConfig->Profile.GetBoolean("/PlanetTool/surface_tracking", false));
    SetNoiseFilterState(pConfig->Profile.GetBoolean("/PlanetTool/noise_filter", false));

    // Enforce valid range limits on planetary detection parameters while restoring from configuration
    m_Planetary_minRadius = pConfig->Profile.GetInt("/PlanetTool/min_radius", PT_MIN_RADIUS_DEFAULT);
    m_Planetary_minRadius = wxMax(PT_RADIUS_MIN, wxMin(PT_RADIUS_MAX, m_Planetary_minRadius));
    m_Planetary_maxRadius = pConfig->Profile.GetInt("/PlanetTool/max_radius", PT_MAX_RADIUS_DEFAULT);
    m_Planetary_maxRadius = wxMax(PT_RADIUS_MIN, wxMin(PT_RADIUS_MAX, m_Planetary_maxRadius));
    m_Planetary_lowThreshold = pConfig->Profile.GetInt("/PlanetTool/high_threshold", PT_HIGH_THRESHOLD_DEFAULT) / 2;
    m_Planetary_lowThreshold = wxMax(PT_THRESHOLD_MIN, wxMin(PT_LOW_THRESHOLD_MAX, m_Planetary_lowThreshold));
    m_Planetary_highThreshold = pConfig->Profile.GetInt("/PlanetTool/high_threshold", PT_HIGH_THRESHOLD_DEFAULT);
    m_Planetary_highThreshold = wxMax(PT_THRESHOLD_MIN, wxMin(PT_HIGH_THRESHOLD_MAX, m_Planetary_highThreshold));
    m_Planetary_minHessian = pConfig->Profile.GetInt("/PlanetTool/min_hessian", PT_MIN_HESSIAN_UI_DEFAULT);
    m_Planetary_minHessian = wxMax(PT_MIN_HESSIAN_MIN, wxMin(PT_MIN_HESSIAN_MAX, m_Planetary_minHessian));
    m_Planetary_maxFeatures = pConfig->Profile.GetInt("/PlanetTool/max_features", PT_MAX_SURFACE_FEATURES);
    m_Planetary_maxFeatures = wxMax(PT_MIN_SURFACE_FEATURES, wxMin(PT_MAX_SURFACE_FEATURES, m_Planetary_maxFeatures));

    // Remove the alert dialog setting for pausing planetary detection
    pConfig->Global.DeleteEntry(PausePlanetDetectionAlertEnabledKey());

    // Initialize non-free OpenCV components
    // Note: this may cause a small and limited memory leak.
    bool nonfreeInit = initModule_nonfree();
    Debug.Write(wxString::Format("OpenCV nonfree module initialization status: %d\n", nonfreeInit));
    assert(nonfreeInit);
}

GuiderPlanet::~GuiderPlanet()
{
    delete m_SER;

    // Save all detection parameters
    pConfig->Profile.SetBoolean("/PlanetTool/surface_tracking", GetSurfaceTrackingState());
    pConfig->Profile.SetBoolean("/PlanetTool/noise_filter", GetNoiseFilterState());
    pConfig->Profile.SetInt("/PlanetTool/min_radius", GetPlanetaryParam_minRadius());
    pConfig->Profile.SetInt("/PlanetTool/max_radius", GetPlanetaryParam_maxRadius());
    pConfig->Profile.SetInt("/PlanetTool/high_threshold", GetPlanetaryParam_highThreshold());
    pConfig->Profile.SetInt("/PlanetTool/min_hessian", GetPlanetaryParam_minHessian());
    pConfig->Profile.SetInt("/PlanetTool/max_features", GetPlanetaryParam_maxFeatures());
    pConfig->Flush();
}

// Planet/feature size depending on planetary detection mode
double GuiderPlanet::GetHFD()
{
    if (m_unknownHFD)
        return std::nan("1");
    if (m_measuringSharpnessMode)
        return m_focusSharpness;
    else
        return m_detected ? m_radius : 0;
}

wxString GuiderPlanet::GetHfdLabel()
{
    if (m_measuringSharpnessMode)
        return _("SHARPNESS: ");
    else
        return _("RADIUS: ");
}

bool GuiderPlanet::IsPixelMetrics()
{
    return GetPlanetaryEnableState() ? !m_measuringSharpnessMode : true;
}

// Handle mouse wheel rotation event from Star Profile windows.
// Positive or negative indicates direction of rotation.
void GuiderPlanet::ZoomStarProfile(int rotation)
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
void GuiderPlanet::ToggleSharpness()
{
    // In surface tracking mode sharpness is always displayed
    if (GetPlanetDetectMode() != PLANET_DETECT_MODE_SURFACE)
    {
        m_measuringSharpnessMode = !m_measuringSharpnessMode;
        m_unknownHFD = true;
    }
}

// The Sobel operator can be used to detect edges in an image, which are more pronounced in
// focused images. You can apply the Sobel operator to the image and calculate the sum or mean
// of the absolute values of the gradients.
double GuiderPlanet::ComputeSobelSharpness(const Mat& img)
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
double GuiderPlanet::CalcSharpness(Mat& FullFrame, Point2f& clickedPoint, bool detectionResult)
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

    const int focusSize = (GetPlanetDetectMode() == PLANET_DETECT_MODE_SURFACE) ? 200 : m_Planetary_maxRadius * 3 / 2.0;
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
void GuiderPlanet::GetDetectionStatus(wxString& statusMsg)
{
    if (GetPlanetDetectMode() == PLANET_DETECT_MODE_SURFACE)
        statusMsg = wxString::Format(_("Object at (%.1f, %.1f)"), m_center_x, m_center_y);
    else
        statusMsg = wxString::Format(_("Object at (%.1f, %.1f) radius=%d"), m_center_x, m_center_y, m_radius);
}

// Update state used to visualize internally detected features
void GuiderPlanet::SetPlanetaryElementsVisual(bool state)
{
    m_syncLock.Lock();
    m_diskContour.clear();
    m_inlierPoints.clear();
    m_Planetary_ShowElementsVisual = state;
    m_syncLock.Unlock();
}

// Notification callback when PHD2 may change CaptureActive state
bool GuiderPlanet::UpdateCaptureState(bool CaptureActive)
{
    bool need_update = false;
    if (m_prevCaptureActive != CaptureActive)
    {
        if (!CaptureActive)
        {
            // Clear selection symbols (green circle/target lock) and visual elements
            if (GetPlanetaryEnableState())
            {
                SetPlanetaryElementsVisual(false);
                pFrame->pGuider->Reset(false);
            }
            m_guidingFixationPointValid = false;
            need_update = true;
        }
        else
        {
            // In planetary tracking mode update the state used to
            // control drawing of the internal detection elements.
            if (GetPlanetaryEnableState() && GetPlanetaryElementsButtonState())
                SetPlanetaryElementsVisual(true);
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
void GuiderPlanet::NotifyCameraConnect(bool connected)
{
    bool isSimCam = (pCamera && pCamera->Name == "Simulator");
    pFrame->pStatsWin->ShowSimulatorStats(isSimCam && connected);
    pFrame->pStatsWin->ShowPlanetStats(GetPlanetaryEnableState() && connected);
    m_roiClicked = false;
}

void GuiderPlanet::SaveCameraSimulationMove(double rx, double ry)
{
    m_cameraSimulationMove = Point2f(rx, ry);
    if (m_simulationZeroOffset)
    {
        m_cameraSimulationRefPoint = m_cameraSimulationMove;
        m_cameraSimulationRefPointValid = true;
    }
}

void GuiderPlanet::RestartSimulatorErrorDetection()
{
    m_cameraSimulationRefPointValid = false;
    m_simulationZeroOffset = true;
}

// Return scaled tracking image with lock target symbol
PHD_Point GuiderPlanet::GetScaledTracker(wxBitmap& scaledBitmap, const PHD_Point& star, double scale)
{
    // Select tracking symbol based on tracking quality
    int targetWidth;
    int targetHeight;
    wxImage* targetImage;
    if (m_trackingQuality > 0)
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

// Helper for visualizing planet detection radius
void GuiderPlanet::PlanetVisualHelper(wxDC& dc, Star primaryStar, double scaleFactor)
{
    // Clip drawing region to displayed image frame
    wxImage* pImg = pFrame->pGuider->DisplayedImage();
    if (pImg)
        dc.SetClippingRegion(wxRect(0, 0, pImg->GetWidth(), pImg->GetHeight()));

    // Make sure to use transparent brush
    dc.SetBrush(*wxTRANSPARENT_BRUSH);

    // Display internally detected elements (must be enabled in UI)
    if (GetPlanetaryElementsVisual())
    {
        m_syncLock.Lock();

        switch (GetPlanetDetectMode())
        {
        case PLANET_DETECT_MODE_SURFACE:
            // Draw detected surface features
            dc.SetPen(wxPen(wxColour(230, 0, 0), 2, wxPENSTYLE_SOLID));
            for (const auto& feature : m_inlierPoints)
            {
                dc.DrawCircle(feature.x * scaleFactor, feature.y * scaleFactor, 5);
            }
            break;
        case PLANET_DETECT_MODE_DISK:
            // Draw contour points in planetary disk mode
            if (m_diskContour.size())
            {
                dc.SetPen(wxPen(wxColour(230, 0, 0), 2, wxPENSTYLE_SOLID));
                for (const Point2f& contourPoint : m_diskContour)
                    dc.DrawCircle((contourPoint.x + m_roiRect.x) * scaleFactor, (contourPoint.y + m_roiRect.y) * scaleFactor, 2);

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
                        dc.DrawCircle((m_sm_circle_x + m_roiRect.x) * scaleFactor, (m_sm_circle_y + m_roiRect.y) * scaleFactor, 3);
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
    if (m_draw_PlanetaryHelper)
    {
        m_draw_PlanetaryHelper = false;
        if (!GetSurfaceTrackingState() && pFrame->CaptureActive)
        {
            const wxString labelTextMin("min diameter");
            const wxString labelTextMax("max diameter");
            int x = int(primaryStar.X * scaleFactor + 0.5);
            int y = int(primaryStar.Y * scaleFactor + 0.5);
            int radius = int(m_radius * scaleFactor + 0.5);
            float minRadius = GetPlanetaryParam_minRadius() * scaleFactor;
            float maxRadius = GetPlanetaryParam_maxRadius() * scaleFactor;
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
            dc.DrawText(labelTextMin, minRadius_x - dc.GetTextExtent(labelTextMin).GetWidth() / 2, y - 10 - dc.GetTextExtent(labelTextMin).GetHeight());

            dc.SetPen(wxPen(wxColour(130, 230, 30), 1, wxPENSTYLE_DOT));
            dc.SetTextForeground(wxColour(130, 230, 30));
            dc.DrawLine(lineMax_x, y + 5, lineMax_x + maxRadius * 2, y + 5);
            dc.DrawCircle(maxRadius_x, y, maxRadius);
            dc.DrawText(labelTextMax, maxRadius_x - dc.GetTextExtent(labelTextMax).GetWidth() / 2, y + 5);
        }
    }
}

void GuiderPlanet::CalcLineParams(CircleDescriptor p1, CircleDescriptor p2)
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
static float CalcContourScore(float& radius, Point2f pointToMeasure, std::vector<Point2f>& diskContour, int minRadius, int maxRadius)
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
    const float* histRange[] = { range };

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
        if (index < ARRAYSIZE(gaussianWeight))
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
    AsyncCalcScoreThread(float bestScore, std::vector<Point2f>& diskContour, std::vector<Point2f>& workLoad, int min_radius, int max_radius)
        : wxThread(wxTHREAD_JOINABLE), threadBestScore(bestScore), contour(diskContour), points(workLoad), minRadius(min_radius), maxRadius(max_radius)
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
int GuiderPlanet::RefineDiskCenter(float& bestScore, CircleDescriptor& diskCenter, std::vector<Point2f>& diskContour, int minRadius, int maxRadius, float searchRadius, float resolution)
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
float GuiderPlanet::FindContourCenter(CircleDescriptor& diskCenter, CircleDescriptor& circle, std::vector<Point2f>& diskContour, Moments& mu, int minRadius, int maxRadius)
{
    float score;
    float maxScore = 0;
    float bestScore = 0;
    float radius = 0;
    int   searchRadius = circle.radius / 2;
    Point2f pointToMeasure;
    std::vector <WeightedCircle> WeightedCircles;
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
        if ((WeightedCircles[i].score > maxScore*0.65) &&
            (WeightedCircles[i].score > WeightedCircles[i - 1].score) &&
            (WeightedCircles[i].score > WeightedCircles[i + 1].score))
        {
            WeightedCircle* localMax = &WeightedCircles[i];
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
void GuiderPlanet::FindCenters(Mat image, const std::vector<Point>& contour, CircleDescriptor& centroid, CircleDescriptor& circle, std::vector<Point2f>& diskContour, Moments& mu, int minRadius, int maxRadius)
{
    const std::vector<Point>* effectiveContour = &contour;
    std::vector<Point> decimatedContour;
    Point2f circleCenter;
    float circle_radius = 0;

    // Add extra margins for min/max radii allowing inclusion of contours
    // outside and inside the given range.
    maxRadius = (maxRadius * 5) / 4;
    minRadius = (minRadius * 3) / 4;

    m_PlanetEccentricity = 0;
    m_PlanetAngle = 0;
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
            m_PlanetEccentricity = sqrt(1 - (minor_axis * minor_axis) / (major_axis * major_axis));

            // Calculate orientation (theta) in radians and convert to degrees
            float theta = 0.5 * atan2(2 * mu.mu11, (mu.mu20 - mu.mu02));
            m_PlanetAngle = theta * (180.0 / CV_PI);
        }
    }
}

// Calculate position of fixation point
Point2f GuiderPlanet::calculateCentroid(const std::vector<KeyPoint>& keypoints, Point2f& clickedPoint)
{
    // If no clicked point is available, calculate centroid of keypoints
    if ((clickedPoint.x == 0) || (clickedPoint.y == 0) || (clickedPoint.x > m_frameWidth) || (clickedPoint.y > m_frameHeight))
    {
        // Calculate centroid of keypoints to be used as a virtual tracking point
        Point2f sum(0, 0);
        for (const auto& kp : keypoints)
            sum += kp.pt;
        // Radius affects scaling factor for the Star Profile window
        m_trackedFeatureSize = TRACKING_FEATURE_SIZE_UNDEF;
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
            if ((dist < minDist) && (kp.pt.x > 50) && (kp.pt.y > 50) && (kp.pt.x < m_frameWidth - 50) && (kp.pt.y < m_frameHeight - 50))
            {
                minDist = dist;
                trackedKeypoint = kp;
                closestPoint = kp.pt;
            }
        }
        // Keypoint size holds diameter of the meaningful keypoint neighborhood
        m_trackedFeatureSize = trackedKeypoint.size;
        m_radius = m_starProfileSize;
        return closestPoint;
    }
}

// Function to check for collinearity
bool GuiderPlanet::areCollinear(const KeyPoint& kp1, const KeyPoint& kp2, const KeyPoint& kp3)
{
    double area2 = abs((kp2.pt.x - kp1.pt.x) * (kp3.pt.y - kp1.pt.y) - (kp3.pt.x - kp1.pt.x) * (kp2.pt.y - kp1.pt.y));
    return area2 < 2.0;  // Consider using a relative threshold based on image dimensions
}

// Function to validate and filter keypoints
bool GuiderPlanet::validateAndFilterKeypoints(std::vector<KeyPoint>& keypoints, std::vector<KeyPoint>& filteredKeypoints)
{
    // Check for sufficient keypoints (at least 4):
    if (keypoints.size() < 4)
    {
        return false; // Indicate insufficient keypoints
    }

    // Filter coincident points:
    double dist_threshold = 5; // Adjust as needed
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
            filteredKeypoints.push_back(keypoints[i]);
    }

    // Filter collinear points:
    for (size_t i = 0; i < filteredKeypoints.size(); ++i)
    {
        for (size_t j = i + 1; j < filteredKeypoints.size(); ++j)
        {
            for (size_t k = j + 1; k < filteredKeypoints.size(); ++k)
            {
                if (areCollinear(filteredKeypoints[i], filteredKeypoints[j], filteredKeypoints[k]))
                {
                    filteredKeypoints.erase(filteredKeypoints.begin() + k);
                    --k; // Adjust index after erasing
                }
            }
        }
    }

    return true; // Indicate successful filtering
}

void GuiderPlanet::SetPlanetaryParam_minHessian(int value)
{
    if (m_Planetary_minHessian != value)
    {
        m_Planetary_minHessian = value;
        m_surfaceDetectionParamsChanging = true;
    }
}
int GuiderPlanet::GetPlanetaryParam_minHessian()
{
    return wxMax(PT_MIN_HESSIAN_UI_MIN, wxMin(m_Planetary_minHessian, PT_MIN_HESSIAN_UI_MAX));
}

// Map the slider value to physical minHessian parameter value using inverse logarithmic scale
int GuiderPlanet::GetPlanetaryParam_minHessianPhysical()
{
    // Ensure the sensitivity value is within the expected range
    int uiSensitivityValue = GetPlanetaryParam_minHessian();
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
bool GuiderPlanet::DetectSurfaceFeatures(Mat image, Point2f& clickedPoint, bool autoSelect)
{
    // No detected features yet
    m_detectedFeatures = 0;

    // Search region for star find is fixed value for surface features tracking
    m_searchRegion = 128;

    // Create SURF detector
    int nOctaves = 4;
    int nOctaveLayers = 2;
    bool upright = true;
    bool surfExtended = false;
    SurfFeatureDetector surfDetector(GetPlanetaryParam_minHessianPhysical(), nOctaves, nOctaveLayers, surfExtended, upright);

    // Enhance local contrast before feature detection
    Mat equalized;
    int cliplimit = 2;
    Ptr<CLAHE> clahe = createCLAHE(cliplimit);
    clahe->apply(image, equalized);

    // Detect keypoints
    std::vector<KeyPoint> keypoints;
    surfDetector.detect(equalized, keypoints);

    // Set locked guiding position when guiding starts
    if (pFrame->pGuider->IsGuiding())
    {
        if (!m_guidingFixationPointValid && m_referenceKeypoints.size())
        {
            m_guidingFixationPoint = m_surfaceFixationPoint;
            m_guidingFixationPointValid = true;
        }
    }
    else
    {
        // While not guiding we can still reset the fixation point based on new clicked point or autoselect
        if (autoSelect || (clickedPoint != m_prevClickedPoint))
            m_referenceKeypoints.clear();
        m_guidingFixationPointValid = false;
    }

    // Exclude keypoints which are too close to frame edges.
    // When setting the reference frame, we limit keypoints to be further away from the edges.
    const int edgeMargin = m_referenceKeypoints.size() ? 15 : 30;
    for (auto it = keypoints.begin(); it != keypoints.end();)
    {
        if ((it->pt.x < edgeMargin) || (it->pt.y < edgeMargin) || (it->pt.x > m_frameWidth - edgeMargin) || (it->pt.y > m_frameHeight - edgeMargin))
            it = keypoints.erase(it);
        else
            ++it;
    }

    // Limit to top N keypoints
    int maxKeypoints = min(m_Planetary_maxFeatures, (int)keypoints.size());
    std::vector<KeyPoint> topKeypoints;
    if (keypoints.size() <= maxKeypoints)
        topKeypoints.assign(keypoints.begin(), keypoints.end());
    else
    {
        // Sort keypoints by response and limit to top N
        std::sort(keypoints.begin(), keypoints.end(),
            [](const KeyPoint& a, const KeyPoint& b)
            { return (a.response > b.response); });
        topKeypoints.assign(keypoints.begin(), keypoints.begin() + maxKeypoints);
    }

    // Filter keypoints
    std::vector<KeyPoint> filteredKeypoints;
    if (!validateAndFilterKeypoints(topKeypoints, filteredKeypoints))
    {
        // Indicate insufficient keypoints
        m_statusMsg = _("No detectable features");
        Debug.Write("Surface feature tracking: " + m_statusMsg + "\n");
        return false;
    }

    // Extract descriptors
    Mat descriptors;
    SurfDescriptorExtractor extractor;
    extractor.compute(equalized, filteredKeypoints, descriptors);

    // When reference keypoints are available, filter and validate keypoints
    if (m_referenceKeypoints.size())
    {
        // Match descriptors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher.knnMatch(m_referenceDescriptors, descriptors, knnMatches, 2);

        // Lowe's Ratio Test : https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
        // This helps to ensure that matches are distinct and likely to be correct.
        std::vector<DMatch> matches;
        matches.reserve(descriptors.rows);
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
            points1.push_back(m_referenceKeypoints[match.queryIdx].pt);
            points2.push_back(filteredKeypoints[match.trainIdx].pt);
        }

        // Find homography using RANSAC
        Mat mask; // This will be filled with the inliers mask
        Mat H = findHomography(points1, points2, CV_RANSAC, 3, mask);

        // Use the mask to filter out the outliers
        Point2f displacement(0, 0);
        std::vector<DMatch> inlierMatches;
        std::vector<Point2f> inlierPoints;
        std::vector<double> distances;
        inlierMatches.reserve(mask.rows);  // Pre-allocate for efficiency
        inlierPoints.reserve(mask.rows);  // Pre-allocate for efficiency
        distances.reserve(mask.rows);  // Pre-allocate for efficiency
        for (size_t i = 0; i < mask.rows; i++)
        {
            if (mask.at<unsigned char>(i))
            {
                auto match = matches[i];
                inlierMatches.push_back(match);
                inlierPoints.push_back(filteredKeypoints[match.trainIdx].pt);

                // Compute average displacement vector between matched keypoints
                displacement += filteredKeypoints[match.trainIdx].pt - m_referenceKeypoints[match.queryIdx].pt;

                // Calculate distances between matched descriptors
                double dist = norm(m_referenceKeypoints[match.queryIdx].pt - filteredKeypoints[match.trainIdx].pt);
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
        double variance = 0.0;
        for (double distance : distances)
        {
            double delta = distance - mean;
            variance += delta * delta;
            mean += delta / distances.size();  // Update mean for unbiased estimator
        }
        variance /= distances.size() - 1;  // Adjust for unbiased estimator

        // Compute average displacement Point2f vector between matched keypoints
        displacement.x /= inlierMatches.size();
        displacement.y /= inlierMatches.size();

        // Calculate new position based on average displacement vector and virtual tracking point
        m_surfaceFixationPoint = m_referencePoint + displacement;

        // Compute distance from the locked guiding position
        double distance = m_guidingFixationPointValid ? norm(m_surfaceFixationPoint - m_guidingFixationPoint) : 0;

        // If variance becomes too high, try to switch to the new reference frame
        const float varianceThreshold = 16.0;
        if ((variance > varianceThreshold) || m_surfaceDetectionParamsChanging)
        {
            // Switch to red tracking box as a warning sign
            m_trackingQuality = 0;
            m_surfaceDetectionParamsChanging = false;

            // When guiding restrict switching to the new reference frame
            if (!pFrame->pGuider->IsGuiding() || ((distance < 100) && (variance < varianceThreshold * 2)))
            {
                // Exclude keypoints which are too close to frame edges.
                // When setting the reference frame, we limit keypoints to be further away from the edges.
                const int edgeMargin = 30;
                for (auto it = filteredKeypoints.begin(); it != filteredKeypoints.end();)
                {
                    if ((it->pt.x < edgeMargin) || (it->pt.y < edgeMargin) || (it->pt.x > m_frameWidth - edgeMargin) || (it->pt.y > m_frameHeight - edgeMargin))
                        it = filteredKeypoints.erase(it);
                    else
                        ++it;
                }
                if (filteredKeypoints.size() < 4)
                {
                    m_statusMsg = _("Too few detectable features");
                    Debug.Write("Surface feature tracking: " + m_statusMsg + "\n");
                    return false;
                }

                // We must recalculate descriptors for the filtered keypoints
                extractor.compute(equalized, filteredKeypoints, descriptors);

                // Set new reference frame. The new position may not be rather accurate
                m_referencePoint = m_surfaceFixationPoint;
                m_referenceKeypoints = filteredKeypoints;
                m_referenceDescriptors = descriptors;
                Debug.Write("Surface feature tracking: update reference point\n");
            }

            // Warn user about unstable image
            if ((variance > 100) && pFrame->pGuider->IsGuiding())
            {
                Debug.Write(wxString::Format("Feature matching encountered very large variance (%.1f), position may not be accurate.\n", variance));
                pFrame->Alert(_("WARNING: image is not stable, tracking may not be accurate!"), wxICON_WARNING);
            }
        }
        else
        {
            // Assess tracking quality based on allowed max distance from the reference frame
            m_trackingQuality = distance < 100 ? 1 : 0;
        }

        // Find the keypoint closest to the surface fixation point
        KeyPoint *trackedKeypoint = NULL;
        float trackedPosMinDistance = 999999.0;
        for (int i = 0; i < filteredKeypoints.size(); ++i)
        {
            double dist = norm(m_surfaceFixationPoint - filteredKeypoints[i].pt);
            if (dist < trackedPosMinDistance)
            {
                trackedPosMinDistance = dist;
                trackedKeypoint = &filteredKeypoints[i];
            }
        }
        // Keypoint size holds diameter of the meaningful keypoint neighborhood
        m_trackedFeatureSize = (trackedPosMinDistance < 15) ? trackedKeypoint->size : TRACKING_FEATURE_SIZE_UNDEF;
        m_radius = m_starProfileSize;

        // Save inlier matches for visualization
        if (GetPlanetaryElementsVisual())
        {
            m_syncLock.Lock();
            m_inlierPoints = inlierPoints;
            m_syncLock.Unlock();
        }

        // Count detected features
        m_detectedFeatures = inlierPoints.size();

        // Update stats
        Debug.Write(wxString::Format("Surface feature tracking: mean=%.1f, variance=%.1f, distance=%.1f, quality=%d, features=%d\n", mean, variance, distance, m_trackingQuality, m_detectedFeatures));
        pFrame->pStatsWin->UpdatePlanetScore(_T("Uncertainty"), variance);
    }
    // Set reference frame keypoints and descriptors
    else if (descriptors.rows > 4)
    {
        // Tracking quality is not available yet
        m_trackingQuality = 0;

        m_referenceKeypoints = filteredKeypoints;
        m_referenceDescriptors = descriptors;

        // Save reference frame centroid
        m_referencePoint = calculateCentroid(filteredKeypoints, clickedPoint);
        m_surfaceFixationPoint = m_referencePoint;

        // Save reference keypoints for visualization
        if (GetPlanetaryElementsVisual())
        {
            m_syncLock.Lock();
            m_inlierPoints.clear();
            m_inlierPoints.reserve(m_referenceKeypoints.size());
            for (const auto& kp : m_referenceKeypoints)
                m_inlierPoints.push_back(kp.pt);
            m_syncLock.Unlock();
        }

        // Count detected features
        m_detectedFeatures = m_referenceKeypoints.size();

        // Assume no more changes to minHessian until further notice
        m_surfaceDetectionParamsChanging = false;
        Debug.Write(wxString::Format("Surface feature tracking: set new reference point, features=%d\n", m_detectedFeatures));
    }

    // Set new object position based on updated centroid
    m_center_x = m_surfaceFixationPoint.x;
    m_center_y = m_surfaceFixationPoint.y;

    return true;
}

// Find planet center using circle matching with contours
bool GuiderPlanet::FindPlanetCenter(Mat img8, int minRadius, int maxRadius, bool roiActive, Point2f& clickedPoint, Rect& roiRect, bool activeRoiLimits, float distanceRoiMax)
{
    int LowThreshold = GetPlanetaryParam_lowThreshold();
    int HighThreshold = GetPlanetaryParam_highThreshold();

    // Apply Canny edge detection
    Debug.Write(wxString::Format("Start detection of planetary disk (roi:%d low_tr=%d,high_tr=%d,minr=%d,maxr=%d)\n", roiActive, LowThreshold, HighThreshold, minRadius, maxRadius));
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
        m_statusMsg = _("Too many contour points detected. Please apply pixel binning, enable ROI, or increase the Edge Detection Threshold.");
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
            float searchRadius = 20 * m_PlanetEccentricity + 3;
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
    Debug.Write(wxString::Format("End detection of planetary disk (t=%d): r=%.1f, x=%.1f, y=%.1f, score=%.3f, contours=%d/%d, threads=%d\n",
        m_PlanetWatchdog.Time(), bestDiskCenter.radius, roiRect.x + bestDiskCenter.x, roiRect.y + bestDiskCenter.y, bestScore, contourMatchingCount, contourAllCount, maxThreadsCount));
    pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Contours/points"), contourMatchingCount, bestContour.size());
    pFrame->pStatsWin->UpdatePlanetScore(("Fitting score"), bestScore);

    // For use by visual aid for parameter tuning
    if (GetPlanetaryElementsVisual())
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
void GuiderPlanet::SaveVideoFrame(cv::Mat& FullFrame, cv::Mat& img8, bool roiActive, int bppFactor)
{
    Mat FullFrame8;
    if (roiActive)
        FullFrame.convertTo(FullFrame8, CV_8U, 1.0 / bppFactor);
    else
        FullFrame8 = img8;

    // Create new SER file on first frame or when frame dimensions change - close previous file first
    if (!m_SER || !m_SER->IsOpen() ||
        (m_SER->FrameWidth() != FullFrame.cols) || (m_SER->FrameHeight() != FullFrame.rows))
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

void GuiderPlanet::UpdateDetectionErrorInSimulator(Point2f& clickedPoint)
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
                pFrame->pStatsWin->UpdatePlanetError(_T("Detection error"), norm(delta - (m_cameraSimulationMove - m_cameraSimulationRefPoint)));
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

// Find planet center of round/crescent shape in the given image
bool GuiderPlanet::FindPlanet(const usImage* pImage, bool autoSelect)
{
    m_PlanetWatchdog.Start();

    // Default error status message
    m_statusMsg = _("Object not found");

    // Skip detection when paused
    if (m_PlanetaryDetectionPaused)
    {
        m_syncLock.Lock();
        m_detected = false;
        m_detectionCounter = 0;
        m_diskContour.clear();
        m_inlierPoints.clear();
        m_syncLock.Unlock();
        return false;
    }

    // Auto select star was requested
    if (autoSelect)
    {
        m_clicked_x = 0;
        m_clicked_y = 0;
        m_roiClicked = false;
        m_detectionCounter = 0;
        RestartSimulatorErrorDetection();
    }
    Point2f clickedPoint = Point2f(m_clicked_x, m_clicked_y);

    // Use ROI for CPU time optimization
    bool roiActive = false;
    int minRadius = (int)GetPlanetaryParam_minRadius();
    int maxRadius = (int)GetPlanetaryParam_maxRadius();
    int roiRadius = (int)(maxRadius * 3 / 2.0 + 0.5);
    int roiOffsetX = 0;
    int roiOffsetY = 0;
    Mat FullFrame(pImage->Size.GetHeight(), pImage->Size.GetWidth(), CV_16UC1, pImage->ImageData);

    // Refuse to process images larger than 4096x4096 and request to use camera binning
    if (FullFrame.cols > 4096 || FullFrame.rows > 4096)
    {
        Debug.Write(wxString::Format("Find planet: image is too large %dx%d\n", FullFrame.cols, FullFrame.rows));
        pFrame->Alert(_("ERROR: camera frame size exceeds maximum limit. Please apply binning to reduce the frame size."), wxICON_ERROR);
        m_syncLock.Lock();
        m_detected = false;
        m_detectionCounter = 0;
        m_diskContour.clear();
        m_inlierPoints.clear();
        m_syncLock.Unlock();
        return false;
    }

    Mat RoiFrame;
    Rect roiRect(0, 0, pImage->Size.GetWidth(), pImage->Size.GetHeight());
    if (!autoSelect && GetRoiEnableState() && m_detected &&
        !GetSurfaceTrackingState() &&
        (m_center_x < m_frameWidth) && (m_center_y < m_frameHeight) &&
        (m_frameWidth == pImage->Size.GetWidth()) && (m_frameHeight == pImage->Size.GetHeight()))
    {
        roiOffsetX = wxMax(0, m_center_x - roiRadius);
        roiOffsetY = wxMax(0, m_center_y - roiRadius);
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
    bool activeRoiLimits = m_roiClicked && GetRoiEnableState();
    float distanceRoiMax = maxRadius * 3 / 2.0;

    bool detectionResult = false;
    try
    {
        // Do slight image blurring to decrease noise impact on results
        Mat imgFiltered;
        GaussianBlur(img8, imgFiltered, cv::Size(3, 3), 1.5);

        // Optional noise suppression filter
        if (GetNoiseFilterState())
        {
            const int d = 10;
            const double sigmaColor = 5.0;
            double sigmaSpace = 5.0;
            Mat filteredImage;
            bilateralFilter(imgFiltered, filteredImage, d, sigmaColor, sigmaSpace);
            imgFiltered = filteredImage;
            Debug.Write(_("Find planet: noise filter applied\n"));
        }

        // Find planet center depending on the selected detection mode
        switch (GetPlanetDetectMode())
        {
        case PLANET_DETECT_MODE_SURFACE:
            detectionResult = DetectSurfaceFeatures(imgFiltered, clickedPoint, autoSelect);
            pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Features"), detectionResult ? m_detectedFeatures : 0);
            break;
        case PLANET_DETECT_MODE_DISK:
            detectionResult = FindPlanetCenter(imgFiltered, minRadius, maxRadius, roiActive, clickedPoint, roiRect, activeRoiLimits, distanceRoiMax);
            break;
        }

        // Calculate sharpness of the image
        if (m_measuringSharpnessMode)
            m_focusSharpness = CalcSharpness(FullFrame, clickedPoint, detectionResult);

        // Update detection time stats
        pFrame->pStatsWin->UpdatePlanetDetectionTime(m_PlanetWatchdog.Time());

        if (detectionResult)
        {
            m_detected = true;
            if (m_detectionCounter++ > 3)
            {
                // Smooth search region to avoid sudden jumps in star find stats
                m_searchRegion = cvRound(m_searchRegion * 0.3 + m_prevSearchRegion * 0.7);

                // Forget about the clicked point after a few successful detections
                m_roiClicked = false;
            }
            m_prevSearchRegion = m_searchRegion;
        }
        if (m_measuringSharpnessMode || detectionResult)
            m_unknownHFD = false;
    }
    catch (const wxString& msg)
    {
        POSSIBLY_UNUSED(msg);
        Debug.Write(wxString::Format("Find planet: exception %s\n", msg));
    }
    catch (const cv::Exception& ex)
    {
        // Handle OpenCV exceptions
        Debug.Write(wxString::Format("Find planet: OpenCV exception %s\n", ex.what()));
        pFrame->Alert(_("ERROR: exception occurred during image processing: change detection parameters"), wxICON_ERROR);
    }
    catch (...)
    {
        // Handle any other exceptions
        Debug.Write("Find planet: unknown exception\n");
        pFrame->Alert(_("ERROR: unknown exception occurred in planetary detection"), wxICON_ERROR);
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
        m_inlierPoints.clear();
    }
    m_roiActive = roiActive;
    m_prevClickedPoint = clickedPoint;
    m_syncLock.Unlock();

    // Check tracking state while guiding
    if (pFrame->pGuider->IsGuiding() && pPointingSource)
    {
        bool tracking = false;
        if (!pPointingSource->GetTracking(&tracking) && !tracking)
        {
            pFrame->Alert("WARNING: mount tracking is disabled!", wxICON_WARNING);
            Debug.Write("Find planet: tracking is disabled during guiding!\n");
        }
    }

    return detectionResult;
}