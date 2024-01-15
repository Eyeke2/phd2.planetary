/*
 *  guider_planetary.h
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

#pragma once

#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/nonfree.hpp"

// Planetary guiding/tracking state and control class
class GuiderPlanet
{

private:
    // Planetary guiding parameters
    bool m_Planetary_enabled;
    bool m_Planetary_SurfaceTracking;
    bool m_EclipseMode;
    bool m_RoiEnabled;
    double m_Planetary_minDist;
    double m_Planetary_param1;
    double m_Planetary_param2;
    double m_Planetary_minRadius;
    double m_Planetary_maxRadius;
    int    m_Planetary_lowThreshold;
    int    m_Planetary_highThreshold;
    int    m_Planetary_minHessian;
    bool   m_Planetary_ShowElementsButtonState;
    bool   m_Planetary_ShowElementsVisual;

    float m_PlanetEccentricity;
    float m_PlanetAngle;

public:
    // Planet detection modes
    enum PlanetDetectMode
    {
        PLANET_DETECT_MODE_CIRCLES = 0,
        PLANET_DETECT_MODE_ECLIPSE = 1,
        PLANET_DETECT_MODE_SURFACE = 2
    };

    bool m_minHessianChanged;
    int m_trackingQuality;
    int m_cachedScaledWidth;
    int m_cachedScaledHeight;
    wxImage* m_cachedTrackerImage;
    wxBitmap m_cachedTrackerScaledBitmap;

    int m_detectedFeatures;
    wxImage m_lockTargetImageOk;
    int m_lockTargetWidthOk;
    int m_lockTargetHeightOk;
    wxImage m_lockTargetImageBad;
    int m_lockTargetWidthBad;
    int m_lockTargetHeightBad;

    wxString m_statusMsg;
    bool m_detected;
    float m_center_x;
    float m_center_y;
    int m_radius;
    int m_frameWidth;
    int m_frameHeight;

    wxMutex m_syncLock;
    bool m_roiActive;
    cv::Rect m_roiRect;
    bool m_roiClicked;
    int m_clicked_x;
    int m_clicked_y;
    cv::Point2f m_prevClickedPoint;

    std::vector<cv::Point2f> m_eclipseContour;
    bool m_circlesValid;
    std::vector<cv::Vec3f> m_circles;
    int m_centoid_x;
    int m_centoid_y;
    int m_sm_circle_x;
    int m_sm_circle_y;
    int m_detectionCounter;

    // Surface tracking
    std::vector<cv::KeyPoint> m_referenceKeypoints;
    cv::Mat m_referenceDescriptors;

    // Reference frame point for surface feature guiding
    cv::Point2f m_referencePoint;

    // Virtual anchor point for for surface feature guiding
    cv::Point2f m_surfaceFixationPoint;

    // Matched inliers for visualizing detected surface features
    std::vector<cv::Point2f> m_inlierPoints;

public:
    GuiderPlanet();

    void CameraConnectNotify() { m_roiClicked = false; };

    PlanetDetectMode GetPlanetDetectMode() const
    {
        if (m_Planetary_SurfaceTracking)
            return PLANET_DETECT_MODE_SURFACE;
        else if (m_EclipseMode)
            return PLANET_DETECT_MODE_ECLIPSE;
        else
            return PLANET_DETECT_MODE_CIRCLES;
    }

    void GetDetectionStatus(wxString& statusMsg);
    void NotifyStartCapturing();
    void NotifyStopCapturing();

    bool GetPlanetaryEnableState() { return m_Planetary_enabled; }
    void SetPlanetaryEnableState(bool enabled) { m_Planetary_enabled = enabled; }
    bool GetSurfaceTrackingState() { return m_Planetary_SurfaceTracking; }
    void SetSurfaceTrackingState(bool enabled) { m_Planetary_SurfaceTracking = enabled; }
    void SetPlanetaryParam_minDist(double val) { m_Planetary_minDist = val; }
    double GetPlanetaryParam_minDist() { return m_Planetary_minDist; }
    void   SetPlanetaryParam_param1(double val) { m_Planetary_param1 = val; }
    double GetPlanetaryParam_param1() { return m_Planetary_param1; }
    void   SetPlanetaryParam_param2(double val) { m_Planetary_param2 = val; }
    double GetPlanetaryParam_param2() { return m_Planetary_param2; }
    void   SetPlanetaryParam_minRadius(double val) { m_Planetary_minRadius = val; }
    double GetPlanetaryParam_minRadius() { return m_Planetary_minRadius; }
    void   SetPlanetaryParam_maxRadius(double val) { m_Planetary_maxRadius = val; }
    double GetPlanetaryParam_maxRadius() { return m_Planetary_maxRadius; }
    bool GetEclipseMode() { return m_EclipseMode; }
    void SetEclipseMode(bool mode) { m_EclipseMode = mode; }
    bool GetRoiEnableState() { return m_RoiEnabled; }
    void SetRoiEnableState(bool enabled) { m_RoiEnabled = enabled; }
    void SetPlanetaryParam_lowThreshold(int value) { m_Planetary_lowThreshold = value; }
    int  GetPlanetaryParam_lowThreshold() { return m_Planetary_lowThreshold; }
    void SetPlanetaryParam_highThreshold(int value) { m_Planetary_highThreshold = value; }
    int  GetPlanetaryParam_highThreshold() { return m_Planetary_highThreshold; }
    void SetPlanetaryParam_minHessian(int value)
    {
        if (m_Planetary_minHessian != value)
        {
            m_Planetary_minHessian = value;
            m_minHessianChanged = true;
        }
    }
    int  GetPlanetaryParam_minHessian() { return m_Planetary_minHessian; }
    int  GetPlanetaryParam_minHessianPhysical();

    void SetPlanetaryElementsVisual(bool state);
    bool GetPlanetaryElementsVisual() { return m_Planetary_ShowElementsVisual; }
    void SetPlanetaryElementsButtonState(bool state) { m_Planetary_ShowElementsButtonState = state; }
    bool GetPlanetaryElementsButtonState() { return m_Planetary_ShowElementsButtonState; }
    PHD_Point GetScaledTracker(wxBitmap& scaledBitmap, const PHD_Point& star, double scale);

public:
    // Displaying visual aid for planetary parameter tuning
    bool m_draw_PlanetaryHelper;
    void PlanetVisualRefresh() { m_draw_PlanetaryHelper = true; }

private:
    wxStopWatch m_PlanetWatchdog;
    typedef struct {
        float x;
        float y;
        float radius;
    } CircleDescriptor;
    struct LineParameters {
        bool  valid;
        bool  vertical;
        float slope;
        float b;
    } m_DiameterLineParameters;
    typedef struct WeightedCircle {
        float x;
        float y;
        float r;
        float score;
    } WeightedCircle;

public:
    void CalcLineParams(CircleDescriptor p1, CircleDescriptor p2);
    int RefineEclipseCenter(float& bestScore, CircleDescriptor& eclipseCenter, std::vector<cv::Point2f>& eclipseContour, int minRadius, int maxRadius, float searchRadius, float resolution = 1.0);
    float FindEclipseCenter(CircleDescriptor& eclipseCenter, CircleDescriptor& smallestCircle, std::vector<cv::Point2f>& bestContourVector, cv::Moments& mu, int minRadius, int maxRadius);
    void FindCenters(cv::Mat image, CvSeq* contours, CircleDescriptor& bestCentroid, CircleDescriptor& smallestCircle, std::vector<cv::Point2f>& bestContour, cv::Moments& mu, int minRadius, int maxRadius);
    bool FindPlanetCircle(cv::Mat img8, int minRadius, int maxRadius, bool roiActive, cv::Point2f& clickedPoint, cv::Rect& roiRect, bool activeRoiLimits, float distanceRoiMax);
    bool FindPlanetEclipse(cv::Mat img8, int minRadius, int maxRadius, bool roiActive, cv::Point2f& clickedPoint, cv::Rect& roiRect, bool activeRoiLimits, float distanceRoiMax);
    bool FindPlanet(const usImage* pImage, bool autoSelect = false);
    void PlanetVisualHelper(wxDC& dc, Star primaryStar, double scaleFactor);

    cv::Point2f calculateCentroid(const std::vector<cv::KeyPoint>& keypoints, cv::Point2f& clickedPoint);
    bool areCollinear(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::KeyPoint& kp3);
    bool validateAndFilterKeypoints(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::KeyPoint>& filteredKeypoints);
    bool DetectSurfaceFeatures(cv::Mat image, cv::Point2f& clickedPoint);
};
