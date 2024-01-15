/*
 *  guider_planetary.cpp
 *  PHD Guiding
 *
 *  Planetary detection extensions by Leo Shatz
 *  Copyright (c) 2023 Leo Shatz
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
    m_Planetary_SurfaceTracking = false;
    m_detected = false;
    m_referenceKeypoints.clear();
    m_inlierPoints.clear();
    m_minHessianChanged = false;
    m_trackingQuality = 0;

    m_cachedScaledWidth = 0;
    m_cachedScaledHeight = 0;
    m_cachedTrackerImage = NULL;

    m_roiClicked = false;
    m_roiActive = false;
    m_detectionCounter = 0;
    m_clicked_x = 0;
    m_clicked_y = 0;
    m_prevClickedPoint = Point2f(0, 0);
    m_circlesValid = false;
    m_eclipseContour.clear();
    m_EclipseMode = false;
    m_RoiEnabled = false;
    m_Planetary_minDist = PT_MIN_DIST_DEFAULT;
    m_Planetary_param1 = PT_PARAM1_DEFAULT;
    m_Planetary_param2 = PT_PARAM2_DEFAULT;
    m_Planetary_minRadius = PT_MIN_RADIUS_DEFAULT;
    m_Planetary_maxRadius = PT_MAX_RADIUS_DEFAULT;
    m_Planetary_lowThreshold = PT_HIGH_THRESHOLD_DEFAULT / 2;
    m_Planetary_highThreshold = PT_HIGH_THRESHOLD_DEFAULT;
    m_Planetary_ShowElementsButtonState = false;
    m_Planetary_ShowElementsVisual = false;
    m_draw_PlanetaryHelper = false;
    m_frameWidth = 0;
    m_frameHeight = 0;
    m_PlanetEccentricity = 0;
    m_PlanetAngle = 0;

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

    // Initialize non-free OpenCV components
    // Note: this may cause a small and limited memory leak.
    bool nonfreeInit = initModule_nonfree();
    Debug.Write(wxString::Format("OpenCV nonfree module initialization status: %d\n", nonfreeInit));
    assert(nonfreeInit);
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
    m_circlesValid = false;
    m_eclipseContour.clear();
    m_inlierPoints.clear();
    m_Planetary_ShowElementsVisual = state;
    m_syncLock.Unlock();
}

// Notification callback about start of capturing
void GuiderPlanet::NotifyStartCapturing()
{
    // In planetary tracking mode update the state used to
    // control drawing of the internal detection elements.
    if (GetPlanetaryEnableState() && GetPlanetaryElementsButtonState())
        SetPlanetaryElementsVisual(true);
}

// Notification callback about stop of capturing
void GuiderPlanet::NotifyStopCapturing()
{
    // In planetary tracking mode we need to redraw the screen
    // without internal detection elements
    if (GetPlanetaryEnableState() && GetPlanetaryElementsVisual())
        SetPlanetaryElementsVisual(false);
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
    // Draw the edge bitmap
    if (GetPlanetaryElementsVisual())
    {
        m_syncLock.Lock();

        // Make sure to use transparent brush
        dc.SetBrush(*wxTRANSPARENT_BRUSH);

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
        case PLANET_DETECT_MODE_CIRCLES:
            // Draw all circles detected by HoughCircles
            if (m_circlesValid)
            {
                dc.SetPen(wxPen(wxColour(230, 0, 0), 2, wxPENSTYLE_SOLID));
                for (const auto& c : m_circles)
                    dc.DrawCircle((m_roiRect.x + c[0]) * scaleFactor, (m_roiRect.y + c[1]) * scaleFactor, c[2] * scaleFactor);
            }
            break;
        case PLANET_DETECT_MODE_ECLIPSE:
            // Draw all edges detected in eclipse mode
            if (m_eclipseContour.size())
            {
                dc.SetPen(wxPen(wxColour(230, 0, 0), 2, wxPENSTYLE_SOLID));
                for (const Point2f& contourPoint : m_eclipseContour)
                    dc.DrawCircle((contourPoint.x + m_roiRect.x) * scaleFactor, (contourPoint.y + m_roiRect.y) * scaleFactor, 2);

#if FILE_SIMULATOR_MODE
                // Draw anchor circle centers
                dc.SetLogicalFunction(wxXOR);
                dc.SetPen(wxPen(wxColour(230, 230, 0), 3, wxPENSTYLE_SOLID));
                if (m_centoid_x && m_centoid_y)
                    dc.DrawCircle((m_centoid_x + m_roiRect.x) * scaleFactor, (m_centoid_y + m_roiRect.y) * scaleFactor, 3);
                dc.SetPen(wxPen(wxColour(230, 230, 0), 1, wxPENSTYLE_SOLID));
                if (m_sm_circle_x && m_sm_circle_y)
                    dc.DrawCircle((m_sm_circle_x + m_roiRect.x) * scaleFactor, (m_sm_circle_y + m_roiRect.y) * scaleFactor, 3);
                dc.SetLogicalFunction(wxCOPY);
#endif
            }
            break;
        }

        m_syncLock.Unlock();
    }

    if (m_draw_PlanetaryHelper)
    {
        if (!GetSurfaceTrackingState())
        {
            m_draw_PlanetaryHelper = false;
            int x = int(primaryStar.X * scaleFactor + 0.5);
            int y = int(primaryStar.Y * scaleFactor + 0.5);

            if (m_detected)
                x -= m_radius * scaleFactor;

            // Draw min and max diameters legend
            const wxString labelTextMin("min diameter");
            const wxString labelTextMax("max diameter");
            dc.SetPen(wxPen(wxColour(230, 130, 30), 1, wxPENSTYLE_DOT));
            dc.SetTextForeground(wxColour(230, 130, 30));
            dc.DrawLine(x, y - 5, x + GetPlanetaryParam_minRadius() * scaleFactor * 2, y - 5);
            dc.DrawText(labelTextMin, x - dc.GetTextExtent(labelTextMin).GetWidth() - 5, y - 10 - dc.GetTextExtent(labelTextMin).GetHeight() / 2);

            dc.SetPen(wxPen(wxColour(130, 230, 30), 1, wxPENSTYLE_DOT));
            dc.SetTextForeground(wxColour(130, 230, 30));
            dc.DrawLine(x, y + 5, x + GetPlanetaryParam_maxRadius() * scaleFactor * 2, y + 5);
            dc.DrawText(labelTextMax, x - dc.GetTextExtent(labelTextMax).GetWidth() - 5, y + 10 - dc.GetTextExtent(labelTextMax).GetHeight() / 2);
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
static float CalcEclipseScore(float& radius, Point2f pointToMeasure, std::vector<Point2f>& eclipseContour, int minRadius, int maxRadius)
{
    const int contourSize = eclipseContour.size();
    float* distances = new float[contourSize];
    float minIt = 0;
    float maxIt = 0;
    int size = 0;

    for (const Point2f& contourPoint : eclipseContour)
    {
        float distance = norm(contourPoint - pointToMeasure);
        if ((distance < minRadius) || (distance > maxRadius))
            continue;
        if (size == 0)
            minIt = maxIt = distance;
        minIt = min(minIt, distance);
        maxIt = max(maxIt, distance);
        distances[size++] = distance;
    }
    for (int x = size; x < contourSize; x++)
        distances[x] = 0;

    // Note: calculating histogram on 0-sized data can crash the application.
    // Reject small sets of points as they usually aren't related to the
    // features we are looking for.
    if (size < 16)
    {
        delete[] distances;
        radius = 0;
        return 0;
    }

    // Calculate the number of bins
    IplImage* distData = cvCreateImageHeader(cvSize(size, 1), IPL_DEPTH_32F, 1);
    cvSetData(distData, distances, size * sizeof(float));
    int bins = int(std::sqrt(size) + 0.5) | 1;
    float range[] = { floor(minIt), ceil(maxIt) };
    float* histRange[] = { range };

    // Calculate the histogram
    CvHistogram* hist = cvCreateHist(1, &bins, CV_HIST_ARRAY, histRange, 1);
    cvCalcHist(&distData, hist, 0, NULL);

    /* Find the peak of the histogram */
    float max_value = 0;
    int max_idx = 0;
    for (int x = 0; x < bins; x++)
    {
        float bin_val = cvQueryHistValue_1D(hist, x);
        if (bin_val > max_value)
        {
            max_value = bin_val;
            max_idx = x;
        }
    }
    // Middle of the bin
    float peakDistance = range[0] + (max_idx + 0.5) * ((range[1] - range[0]) / bins);

    cvReleaseHist(&hist);
    cvReleaseImageHeader(&distData);

    float scorePoints = 0;
    for (int x = 0; x < size; x++)
    {
        int index = fabs(distances[x] - peakDistance) * 100 + 0.5;
        if (index < ARRAYSIZE(gaussianWeight))
            scorePoints += gaussianWeight[index];
    }
    delete[] distances;

    // Normalize score by total number points in the contour
    radius = peakDistance;
    return scorePoints / eclipseContour.size();
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
    AsyncCalcScoreThread(float bestScore, std::vector<Point2f>& eclipseContour, std::vector<Point2f>& workLoad, int min_radius, int max_radius)
        : wxThread(wxTHREAD_JOINABLE), threadBestScore(bestScore), contour(eclipseContour), points(workLoad), minRadius(min_radius), maxRadius(max_radius)
    {
        radius = 0;
    }
    // A thread function to run HoughCircles method
    wxThread::ExitCode Entry()
    {
        for (const Point2f& point : points)
        {
            float score = ::CalcEclipseScore(radius, point, contour, minRadius, maxRadius);
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
int GuiderPlanet::RefineEclipseCenter(float& bestScore, CircleDescriptor& eclipseCenter, std::vector<Point2f>& eclipseContour, int minRadius, int maxRadius, float searchRadius, float resolution)
{
    const int maxWorkloadSize = 256;
    const Point2f center = { eclipseCenter.x, eclipseCenter.y };
    std::vector<AsyncCalcScoreThread *> threads;

    // Check all points within small circle for search of higher score
    int threadCount = 0;
    bool useThreads = true;
    int workloadSize = 0;
    std::vector<Point2f> workload;
    workload.reserve(maxWorkloadSize);
    for (float x = eclipseCenter.x - searchRadius; x < eclipseCenter.x + searchRadius; x += resolution)
        for (float y = eclipseCenter.y - searchRadius; y < eclipseCenter.y + searchRadius; y += resolution)
        {
            Point2f pointToMeasure = { x, y };
            float dist = norm(pointToMeasure - center);
            if (dist > searchRadius)
                continue;

            // When finished crearing a workload, create and run new processing thread
            if (useThreads && (workloadSize++ >= maxWorkloadSize))
            {
                AsyncCalcScoreThread *thread = new AsyncCalcScoreThread(bestScore, eclipseContour, workload, minRadius, maxRadius);
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
                    Debug.Write(_("RefineEclipseCenter: failed to start a thread\n"));
                }
            }
            workload.push_back(pointToMeasure);
        }

    // Process remaining points locally
    for (const Point2f& point : workload)
    {
        float radius;
        float score = ::CalcEclipseScore(radius, point, eclipseContour, minRadius, maxRadius);
        if (score > bestScore)
        {
            bestScore = score;
            eclipseCenter.radius = radius;
            eclipseCenter.x = point.x;
            eclipseCenter.y = point.y;
        }
    }

    // Wait for all threads to terminate and process their results
    for (auto thread : threads)
    {
        thread->Wait();
        if (thread->threadBestScore > bestScore)
        {
            bestScore = thread->threadBestScore;
            eclipseCenter.radius = thread->radius;
            eclipseCenter.x = thread->center.x;
            eclipseCenter.y = thread->center.y;
        }

        delete thread;
    }
    return threadCount;
}

// An algorithm to find eclipse center
float GuiderPlanet::FindEclipseCenter(CircleDescriptor& eclipseCenter, CircleDescriptor& circle, std::vector<Point2f>& eclipseContour, Moments& mu, int minRadius, int maxRadius)
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
        score = CalcEclipseScore(radius, pointToMeasure, eclipseContour, minRadius, maxRadius);
        eclipseCenter = circle;
        eclipseCenter.radius = radius;
        return score;
    }

    if (!m_DiameterLineParameters.vertical && (fabs(m_DiameterLineParameters.slope) <= 1.0))
    {
        // Search along x-axis when line slope is below 45 degrees
        for (pointToMeasure.x = circle.x - searchRadius; pointToMeasure.x <= circle.x + searchRadius; pointToMeasure.x++)
        {
            // Count number of points of the contour which are equidistant from pointToMeasure.
            // The point with maximum score is identified as eclipse center.
            pointToMeasure.y = m_DiameterLineParameters.slope * pointToMeasure.x + m_DiameterLineParameters.b;
            score = CalcEclipseScore(radius, pointToMeasure, eclipseContour, minRadius, maxRadius);
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
            // The point with maximum score is identified as eclipse center.
            if (m_DiameterLineParameters.vertical)
                pointToMeasure.x = circle.x;
            else
                pointToMeasure.x = (pointToMeasure.y - m_DiameterLineParameters.b) / m_DiameterLineParameters.slope;
            score = CalcEclipseScore(radius, pointToMeasure, eclipseContour, minRadius, maxRadius);
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
    eclipseCenter.radius = WeightedCircles[bestIndex].r;
    eclipseCenter.x = WeightedCircles[bestIndex].x;
    eclipseCenter.y = WeightedCircles[bestIndex].y;

    return bestScore;
}

// Find a minimum enclosing circle of the contour and also its center of mass
void GuiderPlanet::FindCenters(Mat image, CvSeq* contour, CircleDescriptor& centroid, CircleDescriptor& circle, std::vector<Point2f>& eclipseContour, Moments& mu, int minRadius, int maxRadius)
{
    CvPoint2D32f circleCenter;
    float circle_radius = 0;

    // Add extra margins for min/max radii allowing inclusion of contours
    // outside and inside the given range.
    maxRadius = (maxRadius * 5) / 4;
    minRadius = (minRadius * 3) / 4;

    m_PlanetEccentricity = 0;
    m_PlanetAngle = 0;
    circle.radius = 0;
    centroid.radius = 0;
    eclipseContour.clear();
    eclipseContour.reserve(contour->total);
    if (cvMinEnclosingCircle(contour, &circleCenter, &circle_radius))
        if ((circle_radius <= maxRadius) &&
            (circle_radius >= minRadius))
        {
            circle.x = circleCenter.x;
            circle.y = circleCenter.y;
            circle.radius = circle_radius;

            // Convert contour to vector of points
            for (int i = 0; i < contour->total; i++)
            {
                CvPoint* pt = CV_GET_SEQ_ELEM(CvPoint, contour, i);
                eclipseContour.push_back(cv::Point(pt->x, pt->y));
            }

            // Calculate center of mass based on contour points
            mu = cv::moments(eclipseContour, false);
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

class AsyncFindCirclesThread : public wxThread
{
public:
    Mat image;
    vector<Vec3f> circles;
    double minDist;
    double param1;
    double param2;
    int minRadius;
    int maxRadius;
    bool active;
    bool finished;

public:
    AsyncFindCirclesThread(Mat img, double min_dist, double param_1, double param_2, int min_radius, int max_radius)
        : wxThread(wxTHREAD_DETACHED), minDist(min_dist), param1(param_1), param2(param_2),
          minRadius(min_radius), maxRadius(max_radius)
        {
            circles.clear();
            image = img.clone();
            active = true;
            finished = false;
        }
    ExitCode Entry() override;
};

// A thread function to run HoughCircles method
wxThread::ExitCode AsyncFindCirclesThread::Entry()
{
    HoughCircles(image, circles, CV_HOUGH_GRADIENT, 1.0, minDist, param1, param2, minRadius, maxRadius);
    finished = true;
    while (active)
        wxMilliSleep(20);
    return this;
}

// Calculate position of fixation point
Point2f GuiderPlanet::calculateCentroid(const std::vector<KeyPoint>& keypoints, Point2f& clickedPoint)
{
    // If no clicked point is available, calculate centroid of keypoints
    if ((clickedPoint.x == 0) || (clickedPoint.y == 0) || (clickedPoint.x > m_frameWidth) || (clickedPoint.y > m_frameHeight))
    {
        // Calculare centroid of keypoints to be used as a virtual tracking point
        Point2f sum(0, 0);
        for (const auto& kp : keypoints)
            sum += kp.pt;
        return Point2f(sum.x / keypoints.size(), sum.y / keypoints.size());
    }
    else
    {
        // Find closest keypoint to clicked point
        double minDist = 999999;
        Point2f closestPoint = Point2f(0, 0);
        for (const auto& kp : keypoints)
        {
            // Calculate distance between clicked point and keypoint but avoid points too close to the edge
            const int margin = 50;
            double dist = norm(kp.pt - clickedPoint);
            if ((dist < minDist) && (kp.pt.x > 50) && (kp.pt.y > 50) && (kp.pt.x < m_frameWidth - 50) && (kp.pt.y < m_frameHeight - 50))
            {
                minDist = dist;
                closestPoint = kp.pt;
            }
        }
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

// Reverse the slider value to get the actual parameter value
int GuiderPlanet::GetPlanetaryParam_minHessianPhysical()
{
    int value = PT_MIN_HESSIAN_MAX - GetPlanetaryParam_minHessian();
    if (value < PT_MIN_HESSIAN_MIN)
        value = PT_MIN_HESSIAN_MIN;
    return value;
}

// Detect/track surface features
bool GuiderPlanet::DetectSurfaceFeatures(Mat image, Point2f& clickedPoint)
{
    // No detected features yet
    m_detectedFeatures = 0;

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

    // While not guiding we can still reset the fixation point based on new clicked point
    if ((pFrame->pGuider->GetState() != STATE_GUIDING) && (clickedPoint != m_prevClickedPoint))
        m_referenceKeypoints.clear();

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

    // Limit to top N keypoints (N = 500)
    int maxKeypoints = min(500, (int)keypoints.size());
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

                // Compute average displacement Point2f vector between matched keypoints
                displacement += filteredKeypoints[match.trainIdx].pt - m_referenceKeypoints[match.queryIdx].pt;

                // Calculate distances between matched descriptors
                // double dist = norm(m_referenceDescriptors.row(match.queryIdx), descriptors.row(match.trainIdx));
                double dist = norm(m_referenceKeypoints[match.queryIdx].pt - filteredKeypoints[match.trainIdx].pt);
                distances.push_back(dist);
            }
        }

        // Discard if very few inliers were found
        if (inlierMatches.size() < 4)
        {
            m_statusMsg = _("Too few detectable features");
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

        // Calculate new centroid based on average displacement vector and virtual tracking point
        m_surfaceFixationPoint = m_referencePoint + displacement;

        // If variance is too high, set new reference frame
        const float varianceThreshold = 16.0;
        if ((variance > varianceThreshold) || m_minHessianChanged)
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
                return false;
            }

            // We must recompute descriptors for the filtered keypoints
            extractor.compute(equalized, filteredKeypoints, descriptors);

            // Set new reference frame -- this is temporary until we have a better way to handle this.
            m_referencePoint = m_surfaceFixationPoint;
            m_referenceKeypoints = filteredKeypoints;
            m_referenceDescriptors = descriptors;

            // The new position won't be rather accurate, but it's better than nothing
            // and it will be updated as soon as we have a better estimate.
            if (variance > varianceThreshold)
                Debug.Write(wxString::Format("Feature matching encountered large variance (%.1f), position may not be accurate.\n", variance));

            // Switch to red tracking box as a warning sign
            m_trackingQuality = 0;
            m_minHessianChanged = false;
        }
        else
        {
            // Switch to green tracking box
            m_trackingQuality = 1;
        }

        // Save inlier matches for visualization
        if (GetPlanetaryElementsVisual())
        {
            m_syncLock.Lock();
            m_inlierPoints = inlierPoints;
            m_syncLock.Unlock();
        }

        // Count detected features
        m_detectedFeatures = inlierPoints.size();
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
        m_minHessianChanged = false;
    }

    // Set new object position based on updated centroid
    m_center_x = m_surfaceFixationPoint.x;
    m_center_y = m_surfaceFixationPoint.y;

    // For surface feature tracking, use a fixed radius
    m_radius = 50;

    return true;
}

// Find planet center and its radius using HoughCircles method
bool GuiderPlanet::FindPlanetCircle(Mat img8, int minRadius, int maxRadius, bool roiActive, Point2f& clickedPoint, Rect& roiRect, bool activeRoiLimits, float distanceRoiMax)
{
    double minDist = GetPlanetaryParam_minDist();
    double param1 = GetPlanetaryParam_param1();
    double param2 = GetPlanetaryParam_param2();

    // Find circles matching given criteria
    Debug.Write(wxString::Format("Start detection of planetary disk (roi:%d mind=%.1f,p1=%.1f,p2=%.1f,minr=%d,maxr=%d)\n", roiActive, minDist, param1, param2, minRadius, maxRadius));

    // We are calling HoughCircles HoughCircles in a separate thread to deal with
    // cases when it takes too long to compure. Under such circumstances,
    // usually caused by small values of minDist / param1 / param2 we stop exposures
    // and report the problem telling the user to increase their values.
    // The hanging thread will eventually terminate and no resource leak
    // or memory corruption should happen as a result.
    vector<Vec3f> circles;
    AsyncFindCirclesThread* thread = new AsyncFindCirclesThread(img8, minDist, param1, param2, minRadius, maxRadius);
    if ((thread->Create() == wxTHREAD_NO_ERROR) && (thread->Run() == wxTHREAD_NO_ERROR))
    {
        const int timeout = 3000;
        while ((m_PlanetWatchdog.Time() < timeout) && !thread->finished)
            wxMilliSleep(20);
        if (!thread->finished)
        {
            Debug.Write(wxString::Format("Detection timeout out, must increase minDist/param1/param2\n"));
            thread->active = false;
            if (pFrame->CaptureActive)
                pFrame->StopCapturing();
            pFrame->m_StopReason = _("Timeout out: exposures stopped! Please increase minDist/param1/param2");
            m_statusMsg = pFrame->m_StopReason;
            return false;
        }
        circles = thread->circles;
        MemoryBarrier();
        thread->active = false;
    }
    else
    {
        delete thread;
        m_statusMsg = _("Internal error: cannot create/run thread.");
        return false;
    }

    // Find and use largest circle from the detected set of circles
    Vec3f center = { 0, 0, 0 };
    for (const auto& c : circles)
    {
        Point2f circlePoint = { roiRect.x + c[0], roiRect.y + c[1] };
        if ((c[2] > center[2]) && (!activeRoiLimits || norm(clickedPoint - circlePoint) <= distanceRoiMax))
            center = c;
    }

    // Save elements to be presented as a visual aid for tunning of the edge threshold parameters.
    if (GetPlanetaryElementsVisual())
    {
        m_syncLock.Lock();
        m_roiRect = roiRect;
        m_circles = circles;
        m_circlesValid = true;
        m_syncLock.Unlock();
    }

    // Log results and update stats window
    Debug.Write(wxString::Format("End detection of planetary disk (t=%d): %d circles detected, r=%d x=%.1f y=%.1f\n", m_PlanetWatchdog.Time(), circles.size(), cvRound(center[2]), center[0], center[1]));
    pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Circles"), circles.size());

    if (center[2])
    {
        m_center_x = roiRect.x + center[0];
        m_center_y = roiRect.y + center[1];
        m_radius = cvRound(center[2]);
        return true;
    }

    return false;
}

// Find planet center using circle matching with contours
bool GuiderPlanet::FindPlanetEclipse(Mat img8, int minRadius, int maxRadius, bool roiActive, Point2f& clickedPoint, Rect& roiRect, bool activeRoiLimits, float distanceRoiMax)
{
    int LowThreshold = GetPlanetaryParam_lowThreshold();
    int HighThreshold = GetPlanetaryParam_highThreshold();

    // Apply Canny edge detection
    Debug.Write(wxString::Format("Start detection of eclipsed disk (roi:%d low_tr=%d,high_tr=%d,minr=%d,maxr=%d)\n", roiActive, LowThreshold, HighThreshold, minRadius, maxRadius));
    Mat edges, dilatedEdges;
    Canny(img8, edges, LowThreshold, HighThreshold, 5, true);
    dilate(edges, dilatedEdges, Mat(), Point(-1, -1), 2);

    // Find contours without interpolation between the points.
    // We need more points for more accurate calculation of the light disk center.
    IplImage thresholded = IplImage(dilatedEdges);
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contours = NULL;
    cvFindContours(&thresholded, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CHAIN_APPROX_NONE);

    // Find total number of contours. If the number is too large, it means that
    // threshold values are too low and we need to increase them.
    int totalPoints = 0;
    for (CvSeq* contour = contours; contour != NULL; contour = contour->h_next)
        totalPoints += contour->total;
    if (totalPoints > 256 * 1024)
    {
        cvReleaseMemStorage(&storage);
        Debug.Write(wxString::Format("Too many contour points detected (%d)\n", totalPoints));
        m_statusMsg = _("Too many contour points detected: please enable ROI or increase Edge Detection Threshold");
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
    CircleDescriptor bestEclipseCenter = { 0 };
    bestContour.clear();
    for (CvSeq* contour = contours; contour != NULL; contour = contour->h_next, contourAllCount++)
    {
        // Ignore contours with small number of points
        if (contour->total < 32)
            continue;

        // Find the smallest circle encompassing contour of the eclipsed disk
        // and also center of mass within the contour.
        cv::Moments mu;
        std::vector<Point2f> eclipseContour;
        CircleDescriptor circle = { 0 };
        CircleDescriptor centroid = { 0 };
        CircleDescriptor eclipseCenter = { 0 };
        FindCenters(img8, contour, centroid, circle, eclipseContour, mu, minRadius, maxRadius);

        // Skip circles not within radius range
        if ((circle.radius == 0) || (eclipseContour.size() == 0))
            continue;

        // Look for a point along the line connecting centers of the smallest circle and center of mass
        // which is most equidistant from the outmost edge of the contour. Consider this point as
        // the best match for eclipse central point.
        CalcLineParams(circle, centroid);
        float score = FindEclipseCenter(eclipseCenter, circle, eclipseContour, mu, minRadius, maxRadius);

        // When user clicks a point in the main window, discard detected features
        // that are far away from it, similar to manual selection of stars in PHD2.
        Point2f circlePoint = { roiRect.x + eclipseCenter.x, roiRect.y + eclipseCenter.y };
        if (activeRoiLimits && (norm(clickedPoint - circlePoint) > distanceRoiMax))
            score = 0;

        /* Refine the best fit */
        if (score > 0.01)
        {
            float searchRadius = 20 * m_PlanetEccentricity + 3;
            RefineEclipseCenter(score, eclipseCenter, eclipseContour, minRadius, maxRadius, searchRadius);
            if (score > bestScore * 0.8)
                RefineEclipseCenter(score, eclipseCenter, eclipseContour, minRadius, maxRadius, 0.5, 0.1);
        }

        // Select best fit based on highest score
        if (score > bestScore)
        {
            bestScore = score;
            bestEclipseCenter = eclipseCenter;
            bestCentroid = centroid;
            bestContour = eclipseContour;
            bestCircle = circle;
        }
        contourMatchingCount++;
    }

    // Free allocated storage
    cvReleaseMemStorage(&storage);
    Debug.Write(wxString::Format("End detection of eclipsed disk (t=%d): r=%.1f, x=%.1f, y=%.1f, score=%.3f, contours=%d/%d\n",
        m_PlanetWatchdog.Time(), bestEclipseCenter.radius, roiRect.x + bestEclipseCenter.x, roiRect.y + bestEclipseCenter.y, bestScore, contourMatchingCount, contourAllCount));

    // Update stats window
    pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Contour points"), totalPoints);

    // Create wxImage from the OpenCV Mat to be presented as
    // a visual aid for tunning of the edge threshold parameters.
    if (GetPlanetaryElementsVisual())
    {
        m_syncLock.Lock();
        m_roiRect = roiRect;
        m_eclipseContour = bestContour;
        m_centoid_x = bestCentroid.x;
        m_centoid_y = bestCentroid.y;
        m_sm_circle_x = bestCircle.x;
        m_sm_circle_y = bestCircle.y;
        m_syncLock.Unlock();
    }

    if (bestEclipseCenter.radius > 0)
    {
        m_center_x = roiRect.x + bestEclipseCenter.x;
        m_center_y = roiRect.y + bestEclipseCenter.y;
        m_radius = cvRound(bestEclipseCenter.radius);
        return true;
    }

    return false;
}

// Find planet center of round/crescent shape in the given image
bool GuiderPlanet::FindPlanet(const usImage *pImage, bool autoSelect)
{
    m_PlanetWatchdog.Start();

    // Default error status message
    m_statusMsg = _("Object not found");

    // Point clicked by user in the main window
    Point2f clickedPoint = { (float) m_clicked_x, (float)m_clicked_y };
    if (autoSelect)
        m_roiClicked = false;

    // Make sure to use 8-bit gray image for feature detection
    int format;
    switch (pImage->BitsPerPixel)
    {
    case 16: format = CV_16UC1;
        break;
    case 8: format = CV_8UC1;
        break;
    default:
        m_detected = false;
        m_circlesValid = false;
        m_detectionCounter = 0;
        return false;
    }

    int minRadius = (int) GetPlanetaryParam_minRadius();
    int maxRadius = (int) GetPlanetaryParam_maxRadius();
    int roiRadius = (int) (maxRadius * 3 / 2.0 + 0.5);

    // Use ROI for CPU time optimization
    bool roiActive = false;
    int roiOffsetX = 0;
    int roiOffsetY = 0;
    Mat FullFrame(pImage->Size.GetHeight(), pImage->Size.GetWidth(), format, pImage->ImageData);
    Mat RoiFrame;
    Mat img8;
    Rect roiRect(0, 0, pImage->Size.GetWidth(), pImage->Size.GetHeight());
    if (!autoSelect && GetRoiEnableState() && m_detected &&
        !GetSurfaceTrackingState() &&
        (m_center_x < m_frameWidth) &&
        (m_center_y < m_frameHeight) &&
        (m_frameWidth == pImage->Size.GetWidth()) &&
        (m_frameHeight == pImage->Size.GetHeight()))
    {
        roiOffsetX = m_center_x - roiRadius;
        roiOffsetX = max(0, roiOffsetX);
        roiOffsetY = m_center_y - roiRadius;
        roiOffsetY = max(0, roiOffsetY);
        int w = roiRadius * 2;
        w = min(w, pImage->Size.GetWidth() - roiOffsetX);
        int h = roiRadius * 2;
        h = min(h, pImage->Size.GetHeight() - roiOffsetY);
        roiRect = Rect(roiOffsetX, roiOffsetY, w, h);
        RoiFrame = FullFrame(roiRect);
        roiActive = true;
    }
    else
        RoiFrame = FullFrame;
    if (pImage->BitsPerPixel == 16)
        RoiFrame.convertTo(img8, CV_8U, 1.0 / 256.0);
    else
        img8 = RoiFrame;

    // Save latest frame dimensions
    m_frameWidth = pImage->Size.GetWidth();
    m_frameHeight = pImage->Size.GetHeight();

    // ROI current state and limit
    bool activeRoiLimits = m_roiClicked && GetRoiEnableState();
    float distanceRoiMax = maxRadius * 3 / 2.0;

    bool detectionResult = false;
    try
    {
        // Do slight image bluring to decrease noise impact on results
        GaussianBlur(img8, img8, cv::Size(3, 3), 1.5);

        // Find planet center depending on the selected detection mode
        switch (GetPlanetDetectMode())
        {
            case PLANET_DETECT_MODE_SURFACE:
                detectionResult = DetectSurfaceFeatures(img8, clickedPoint);
                pFrame->pStatsWin->UpdatePlanetFeatureCount(_T("Features"), detectionResult ? m_detectedFeatures : 0);
                break;
            case PLANET_DETECT_MODE_CIRCLES:
                detectionResult = FindPlanetCircle(img8, minRadius, maxRadius, roiActive, clickedPoint, roiRect, activeRoiLimits, distanceRoiMax);
                break;
            case PLANET_DETECT_MODE_ECLIPSE:
                detectionResult = FindPlanetEclipse(img8, minRadius, maxRadius, roiActive, clickedPoint, roiRect, activeRoiLimits, distanceRoiMax);
                break;
        }

        // Update detection time stats
        pFrame->pStatsWin->UpdatePlanetDetectionTime(m_PlanetWatchdog.Time());

        if (detectionResult)
        {
            m_detected = true;
            if (m_detectionCounter++ > 3)
                m_roiClicked = false;
        }
    }
    catch (const wxString& msg)
    {
        POSSIBLY_UNUSED(msg);
        Debug.Write(wxString::Format("Find planet: exception %s\n", msg));
    }

    // Update data shared with other thread
    m_syncLock.Lock();
    m_roiRect = roiRect;
    if (!detectionResult)
    {
        m_detected = false;
        m_detectionCounter = 0;
        m_eclipseContour.clear();
        m_inlierPoints.clear();
        m_circlesValid = false;
    }
    m_roiActive = roiActive;
    m_prevClickedPoint = clickedPoint;
    m_syncLock.Unlock();

    return detectionResult;
}