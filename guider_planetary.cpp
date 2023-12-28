/*
 *  guider_planetary.cpp
 *  PHD Guiding

 *  Original guider_onestar Created by Craig Stark.
 *  Copyright (c) 2006-2010 Craig Stark.
 *  All rights reserved.
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

#include <wx/dir.h>
#include <algorithm>

#if ((wxMAJOR_VERSION < 3) && (wxMINOR_VERSION < 9))
#define wxPENSTYLE_DOT wxDOT
#endif

using namespace cv;

// Gaussian weights lookup table
static float gaussianWeight[2000];

// Initialize planetary module
void GuiderMultiStar::InitPlanetaryModule()
{
    // Build gaussian weighting function table used for circle feature detection
    float sigma = 1.0;
    memset(gaussianWeight, 0, sizeof(gaussianWeight));
    for (double x = 0; x < 20; x += 0.01)
    {
        int i = x * 100 + 0.5;
        if (i < ARRAYSIZE(gaussianWeight))
            gaussianWeight[i] += exp(-(pow(x, 2) / (2 * pow(sigma, 2))));
    }
    m_PlanetEccentricity = 0;
    m_PlanetAngle = 0;
}

// Helper for visualizing planet detection radius
void GuiderMultiStar::PlanetVisualHelper(wxDC& dc)
{
    // Draw the edge bitmap
    if (GetPlanetaryElementsVisual())
    {
        m_Planet.sync_lock.Lock();

        // Make sure to use transparent brush
        dc.SetBrush(*wxTRANSPARENT_BRUSH);

        // Draw all edges detected in eclipse mode
        if (GetEclipseMode() && m_Planet.eclipseContour.size())
        {
            dc.SetPen(wxPen(wxColour(230, 0, 0), 2, wxPENSTYLE_SOLID));
            for (const Point2f& contourPoint : m_Planet.eclipseContour)
                dc.DrawCircle((contourPoint.x + m_Planet.offset_x) * m_scaleFactor, (contourPoint.y + m_Planet.offset_y) * m_scaleFactor, 2);

#if FILE_SIMULATOR_MODE
            // Draw ancor circle centers
            dc.SetLogicalFunction(wxXOR);
            dc.SetPen(wxPen(wxColour(230, 230, 0), 3, wxPENSTYLE_SOLID));
            if (m_Planet.centoid_x && m_Planet.centoid_y)
                dc.DrawCircle((m_Planet.centoid_x + m_Planet.offset_x) * m_scaleFactor, (m_Planet.centoid_y + m_Planet.offset_y) * m_scaleFactor, 3);
            dc.SetPen(wxPen(wxColour(230, 230, 0), 1, wxPENSTYLE_SOLID));
            if (m_Planet.sm_circle_x && m_Planet.sm_circle_y)
                dc.DrawCircle((m_Planet.sm_circle_x + m_Planet.offset_x) * m_scaleFactor, (m_Planet.sm_circle_y + m_Planet.offset_y) * m_scaleFactor, 3);
            dc.SetLogicalFunction(wxCOPY);
#endif
        }

        // Draw all circles detected by HoughCircles
        if (!GetEclipseMode() && m_Planet.circles_valid)
        {
            dc.SetPen(wxPen(wxColour(230, 0, 0), 2, wxPENSTYLE_SOLID));
            for (const auto& c : m_Planet.circles)
                dc.DrawCircle((m_Planet.offset_x + c[0]) * m_scaleFactor, (m_Planet.offset_y + c[1]) * m_scaleFactor, c[2] * m_scaleFactor);
        }

        m_Planet.sync_lock.Unlock();
    }

    if (m_draw_PlanetaryHelper)
    {
        m_draw_PlanetaryHelper = false;
        int x = int(m_primaryStar.X * m_scaleFactor + 0.5);
        int y = int(m_primaryStar.Y * m_scaleFactor + 0.5);

        if (m_Planet.detected)
            x -= m_Planet.radius * m_scaleFactor;

        // Draw min and max diameters legend
        const wxString labelTextMin("min diameter");
        const wxString labelTextMax("max diameter");
        dc.SetPen(wxPen(wxColour(230, 130, 30), 1, wxPENSTYLE_DOT));
        dc.SetTextForeground(wxColour(230, 130, 30));
        dc.DrawLine(x, y - 5, x + GetPlanetaryParam_minRadius() * m_scaleFactor * 2, y - 5);
        dc.DrawText(labelTextMin, x - dc.GetTextExtent(labelTextMin).GetWidth() - 5, y - 10 - dc.GetTextExtent(labelTextMin).GetHeight() / 2);

        dc.SetPen(wxPen(wxColour(130, 230, 30), 1, wxPENSTYLE_DOT));
        dc.SetTextForeground(wxColour(130, 230, 30));
        dc.DrawLine(x, y + 5, x + GetPlanetaryParam_maxRadius() * m_scaleFactor * 2, y + 5);
        dc.DrawText(labelTextMax, x - dc.GetTextExtent(labelTextMax).GetWidth() - 5, y + 10 - dc.GetTextExtent(labelTextMax).GetHeight() / 2);
    }
}

void GuiderMultiStar::CalcLineParams(CircleDescriptor p1, CircleDescriptor p2)
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
    float* distances = new float[eclipseContour.size()];
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
    for (int x = size; x < eclipseContour.size(); x++)
        distances[x] = 0;

    // Note: calculating histogram on 0-sized data can crash the application.
    // Reject small sets of points as they usually aren't related to the
    // features we are looking for.
    if (size < 16)
    {
        delete distances;
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
    delete distances;

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
int GuiderMultiStar::RefineEclipseCenter(float& bestScore, CircleDescriptor& eclipseCenter, std::vector<Point2f>& eclipseContour, int minRadius, int maxRadius, float searchRadius, float resolution)
{
    const int maxWorkloadSize = 256;
    const Point2f center = { eclipseCenter.x, eclipseCenter.y };
    std::vector<AsyncCalcScoreThread *> threads;

    // Check all points within small circle for search of higher score
    int threadCount = 0;
    bool useThreads = true;
    int workloadSize = 0;
    std::vector<Point2f> workload;
    for (float x = eclipseCenter.x - searchRadius; x < eclipseCenter.x + searchRadius; x += resolution)
        for (float y = eclipseCenter.y - searchRadius; y < eclipseCenter.y + searchRadius; y += resolution)
        {
            float radius;
            Point2f pointToMeasure = { x, y };
            float dist = norm(pointToMeasure - center);
            if (dist > searchRadius)
                continue;

            // When finished crearing a workload, create and run new processing thread
            if (useThreads && (workloadSize++ > maxWorkloadSize))
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
float GuiderMultiStar::FindEclipseCenter(CircleDescriptor& eclipseCenter, CircleDescriptor& circle, std::vector<Point2f>& eclipseContour, Moments& mu, int minRadius, int maxRadius)
{
    float score;
    float maxScore = 0;
    float bestScore = 0;
    float radius = 0;
    int   searchRadius = circle.radius / 2;
    Point2f pointToMeasure;
    std::vector <WeightedCircle> WeightedCircles;

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
void GuiderMultiStar::FindCenters(Mat image, CvSeq* contour, CircleDescriptor& centroid, CircleDescriptor& circle, std::vector<Point2f>& eclipseContour, Moments& mu, int minRadius, int maxRadius)
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
        wxMilliSleep(1);
    return this;
}

bool GuiderMultiStar::FindPlanet(const usImage *pImage, bool autoSelect)
{
    m_PlanetWatchdog.Start();

    Point2f clickedPoint = { (float) m_Planet.clicked_x, (float) m_Planet.clicked_y };
    if (autoSelect)
        m_Planet.clicked = false;

    // Make sure to use 8-bit gray image for feature detection
    int format;
    switch (pImage->BitsPerPixel)
    {
    case 16: format = CV_16UC1;
        break;
    case 8: format = CV_8UC1;
        break;
    default:
        m_Planet.detected = false;
        m_Planet.circles_valid = false;
        return false;
    }

    // Use ROI as optimization
    bool usingRoi = false;
    int roiOffsetX = 0;
    int roiOffsetY = 0;
    Mat FullFrame(pImage->Size.GetHeight(), pImage->Size.GetWidth(), format, pImage->ImageData);
    Mat RoiFrame;
    Mat img8;
    if (!autoSelect && GetRoiEnableState() &&
        (m_Planet.center_x < m_Planet.frame_width) &&
        (m_Planet.center_y < m_Planet.frame_height) &&
        (m_Planet.roi_radius > 0) &&
        (m_Planet.frame_width == pImage->Size.GetWidth()) &&
        (m_Planet.frame_height == pImage->Size.GetHeight()))
    {
        roiOffsetX = m_Planet.center_x - m_Planet.roi_radius;
        roiOffsetX = max(0, roiOffsetX);
        roiOffsetY = m_Planet.center_y - m_Planet.roi_radius;
        roiOffsetY = max(0, roiOffsetY);
        int w = m_Planet.roi_radius * 2;
        w = min(w, pImage->Size.GetWidth() - roiOffsetX);
        int h = m_Planet.roi_radius * 2;
        h = min(h, pImage->Size.GetHeight() - roiOffsetY);
        Rect roiRect(roiOffsetX, roiOffsetY, w, h);
        RoiFrame = FullFrame(roiRect);
        usingRoi = true;
    }
    else
        RoiFrame = FullFrame;
    if (pImage->BitsPerPixel == 16)
        RoiFrame.convertTo(img8, CV_8U, 1.0 / 256.0);
    else
        img8 = RoiFrame;

    // Do slight image bluring to decrease noise impact on results
    cv::medianBlur(img8, img8, 5);

    int minRadius = (int) GetPlanetaryParam_minRadius();
    int maxRadius = (int) GetPlanetaryParam_maxRadius();

    try {

        bool eclipse_mode = GetEclipseMode();
        if (!eclipse_mode)
        {
            double minDist = GetPlanetaryParam_minDist();
            double param1 = GetPlanetaryParam_param1();
            double param2 = GetPlanetaryParam_param2();

            // Find circles matching given criteria
            Debug.Write(wxString::Format("Start detection of planetary disk (roi:%d mind=%.1f,p1=%.1f,p2=%.1f,minr=%d,maxr=%d)\n", usingRoi, minDist, param1, param2, minRadius, maxRadius));

            // We are calling HoughCircles HoughCircles in a separate thread to deal with
            // cases when it takes too long to compure. Under such circumstances,
            // usually caused by small values of param1 / param2 we stop exposures
            // and report the problem telling the user to increase their values.
            // The hanging thread will eventually terminate and no resource leak
            // or memory corruption should happen as a result.
            vector<Vec3f> circles;
            AsyncFindCirclesThread* thread = new AsyncFindCirclesThread(img8, minDist, param1, param2, minRadius, maxRadius);
            if ((thread->Create() == wxTHREAD_NO_ERROR) && (thread->Run() == wxTHREAD_NO_ERROR))
            {
                const int timeout = 2000;
                int msec;
                for (msec = 0; msec < timeout && !thread->finished; msec += 1)
                    wxMilliSleep(1);
                if (msec >= timeout && !thread->finished)
                {
                    Debug.Write(wxString::Format("Detection timeout out, must increase param1/param2\n"));
                    thread->active = false;
                    m_Planet.detected = false;
                    m_Planet.roi_radius = 0;
                    if (pFrame->CaptureActive)
                        pFrame->StopCapturing();
                    pFrame->m_StopReason = _("Timeout out: must increase param1/param2! Stopped.");
                    m_Planet.circles_valid = false;
                    return false;
                }
                circles = thread->circles;
                MemoryBarrier();
                thread->active = false;
            }
            else
            {
                delete thread;
                m_Planet.detected = false;
                m_Planet.roi_radius = 0;
                return false;
            }

            // Find and use largest circle from the detected set of circles
            Vec3f center = { 0, 0, 0 };
            for (const auto& c : circles)
            {
                Point2f circlePoint = { roiOffsetX + c[0], roiOffsetY + c[1] };
                float distanceAllowed = c[2] * 1.5;
                if ((c[2] > center[2]) && (!m_Planet.clicked || autoSelect || norm(clickedPoint - circlePoint) <= distanceAllowed))
                    center = c;
            }

            // Save elements to be presented as a visual aid for tunning of the edge threshold parameters.
            if (GetPlanetaryElementsVisual())
            {
                m_Planet.sync_lock.Lock();
                m_Planet.offset_x = roiOffsetX;
                m_Planet.offset_y = roiOffsetY;
                m_Planet.circles = circles;
                m_Planet.circles_valid = true;
                m_Planet.sync_lock.Unlock();
            }

            Debug.Write(wxString::Format("End detection of planetary disk (t=%d): %d circles detected, r=%d x=%.1f y=%.1f\n", m_PlanetWatchdog.Time(), circles.size(), cvRound(center[2]), center[0], center[1]));
            if (center[2])
            {
                m_Planet.frame_width = pImage->Size.GetWidth();
                m_Planet.frame_height = pImage->Size.GetHeight();
                m_Planet.center_x = roiOffsetX + center[0];
                m_Planet.center_y = roiOffsetY + center[1];
                m_Planet.radius = cvRound(center[2]);
                m_Planet.roi_radius = (m_Planet.radius * 3) / 2;
                m_Planet.detected = true;
                return true;
            }
        }
        else
        {
            int LowThreshold = GetPlanetaryParam_lowThreshold();
            int HighThreshold = GetPlanetaryParam_highThreshold();

            // Apply Canny edge detection
            Debug.Write(wxString::Format("Start detection of eclipsed disk (roi:%d low_tr=%d,high_tr=%d,minr=%d,maxr=%d)\n", usingRoi, LowThreshold, HighThreshold, minRadius, maxRadius));
            Mat edges, dilatedEdges;
            Canny(img8, edges, LowThreshold, HighThreshold, 5, true);
            dilate(edges, dilatedEdges, Mat(), Point(-1, -1), 2);

            // Find contours without interpolation between the points.
            // We need more points for more accurate calculation of the light disk center.
            IplImage thresholded = IplImage(dilatedEdges);
            CvMemStorage* storage = cvCreateMemStorage(0);
            CvSeq* contours = NULL;
            cvFindContours(&thresholded, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CHAIN_APPROX_NONE);

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
                Point2f circlePoint = { roiOffsetX + eclipseCenter.x, roiOffsetY + eclipseCenter.y };
                float distanceAllowed = eclipseCenter.radius * 1.5;
                if (!autoSelect && m_Planet.clicked && (eclipseCenter.radius > 0) && norm(clickedPoint - circlePoint) > distanceAllowed)
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
                m_PlanetWatchdog.Time(), bestEclipseCenter.radius, roiOffsetX + bestEclipseCenter.x, roiOffsetY + bestEclipseCenter.y, bestScore, contourMatchingCount, contourAllCount));

            // Create wxImage from the OpenCV Mat to be presented as
            // a visual aid for tunning of the edge threshold parameters.
            if (GetPlanetaryElementsVisual())
            {
                m_Planet.sync_lock.Lock();
                m_Planet.eclipseContour = bestContour;
                m_Planet.offset_x = roiOffsetX;
                m_Planet.offset_y = roiOffsetY;
                m_Planet.centoid_x = bestCentroid.x;
                m_Planet.centoid_y = bestCentroid.y;
                m_Planet.sm_circle_x = bestCircle.x;
                m_Planet.sm_circle_y = bestCircle.y;
                m_Planet.sync_lock.Unlock();
            }

            if (bestEclipseCenter.radius > 0)
            {
                m_Planet.frame_width = pImage->Size.GetWidth();
                m_Planet.frame_height = pImage->Size.GetHeight();
                m_Planet.center_x = roiOffsetX + bestEclipseCenter.x;
                m_Planet.center_y = roiOffsetY + bestEclipseCenter.y;
                m_Planet.radius = bestEclipseCenter.radius;
                m_Planet.roi_radius = (m_Planet.radius * 3) / 2;
                m_Planet.detected = true;
                return true;
            }
        }
    }
    catch (const wxString& msg)
    {
        POSSIBLY_UNUSED(msg);
        Debug.Write(wxString::Format("Find planet: exception %s\n", msg));
    }

    m_Planet.detected = false;
    m_Planet.roi_radius = 0;
    return false;
}