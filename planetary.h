/*
 *  planetary.h
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

#pragma once

#include "opencv/cv.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "ser_file.h"

// Marks undefined feature size
#define TRACKING_FEATURE_SIZE_UNDEF 999.99

// Solar, lunar and planetary detection state and control class
class SolarSystemObject
{
private:
    // Solar system object guiding parameters
    bool   m_paramEnabled;
    bool   m_paramDetectionPaused;
    bool   m_paramSurfaceTracking;
    bool   m_paramRoiEnabled;

    double m_paramMinRadius;
    double m_paramMaxRadius;
    int    m_paramLowThreshold;
    int    m_paramHighThreshold;
    int    m_paramMinHessian;
    bool   m_paramShowElementsButtonState;
    bool   m_paramNoiseFilterState;
    int    m_paramMaxFeatures;

    bool   m_showVisualElements;
    bool   m_prevCaptureActive;
    bool   m_measuringSharpnessMode;
    bool   m_unknownHFD;
    double m_focusSharpness;
    int    m_starProfileSize;

    float  m_eccentricity;
    float  m_angle;

    bool m_surfaceDetectionParamsChanging;
    int  m_trackingQuality;
    int  m_cachedScaledWidth;
    int  m_cachedScaledHeight;
    wxImage* m_cachedTrackerImage;
    wxBitmap m_cachedTrackerScaledBitmap;

    double m_trackedFeatureSize;
    int m_detectedFeatures;
    wxImage m_lockTargetImageOk;
    int m_lockTargetWidthOk;
    int m_lockTargetHeightOk;
    wxImage m_lockTargetImageBad;
    int m_lockTargetWidthBad;
    int m_lockTargetHeightBad;

    wxMutex m_syncLock;
    cv::Point2f m_prevClickedPoint;

    std::vector<cv::Point2f> m_diskContour;
    int m_centoid_x;
    int m_centoid_y;
    int m_sm_circle_x;
    int m_sm_circle_y;
    int m_frameWidth;
    int m_frameHeight;

    cv::Point2f m_origPoint;
    cv::Point2f m_cameraSimulationMove;
    cv::Point2f m_cameraSimulationRefPoint;

    // Surface tracking
    std::vector<cv::KeyPoint> m_referenceKeypoints;
    cv::Mat m_referenceDescriptors;

    // Reference frame point for surface feature guiding
    cv::Point2f m_referencePoint;

    // Virtual anchor point for for surface feature guiding
    cv::Point2f m_surfaceFixationPoint;
    cv::Point2f m_guidingFixationPoint;
    bool m_guidingFixationPointValid;

    // Matched inliers for visualizing detected surface features
    std::vector<cv::Point2f> m_inlierPoints;

    // SER file currently used for logging image frames
    bool m_videoLogEnabled;
    SERFile *m_SER;

public:
    // Solar system object detection modes
    enum DetectionMode
    {
        DETECTION_MODE_DISK = 1,
        DETECTION_MODE_SURFACE = 2
    };

    wxString m_statusMsg;
    bool m_detected;
    float m_center_x;
    float m_center_y;
    int m_radius;
    int m_searchRegion;
    float m_prevSearchRegion;

    bool m_roiActive;
    cv::Rect m_roiRect;
    bool m_userLClick;
    int m_clicked_x;
    int m_clicked_y;

    int m_detectionCounter;
    bool m_simulationZeroOffset;
    bool m_cameraSimulationRefPointValid;

    // PHD2 parameters saved before enabling solar system object mode and restored after disabling
    bool m_phd2_MassChangeThresholdEnabled;
    bool m_phd2_UseSubframes;
    bool m_phd2_MultistarEnabled;

public:
    SolarSystemObject();
    ~SolarSystemObject();

    bool FindSolarSystemObject(const usImage* pImage, bool autoSelect = false);
    void RestartSimulatorErrorDetection();

    PHD_Point GetScaledTracker(wxBitmap& scaledBitmap, const PHD_Point& star, double scale);

    DetectionMode GetPlanetDetectMode() const
    {
        if (m_paramSurfaceTracking)
            return DETECTION_MODE_SURFACE;
        else
            return DETECTION_MODE_DISK;
    }

    double GetHFD();
    wxString GetHfdLabel();
    bool IsPixelMetrics();
    void ZoomStarProfile(int rotation);
    void ToggleSharpness();
    void GetDetectionStatus(wxString& statusMsg);
    void NotifyCameraConnect(bool connected);
    bool UpdateCaptureState(bool CaptureActive);
    void SaveCameraSimulationMove(double rx, double ry);

    bool Get_SolarSystemObjMode() { return m_paramEnabled; }
    void Set_SolarSystemObjMode(bool enabled) { m_paramEnabled = enabled; }
    bool GetDetectionPausedState() { return m_paramDetectionPaused; }
    void SetDetectionPausedState(bool paused) { m_paramDetectionPaused = paused; }
    bool GetSurfaceTrackingState() { return m_paramSurfaceTracking; }
    void SetSurfaceTrackingState(bool enabled)
    {
        m_paramSurfaceTracking = enabled;
        m_measuringSharpnessMode = enabled;
        m_unknownHFD = true;
    }
    void   Set_minRadius(double val) { m_paramMinRadius = val; }
    double Get_minRadius() { return m_paramMinRadius; }
    void   Set_maxRadius(double val) { m_paramMaxRadius = val; }
    double Get_maxRadius() { return m_paramMaxRadius; }
    bool GetRoiEnableState() { return m_paramRoiEnabled; }
    void SetRoiEnableState(bool enabled) { m_paramRoiEnabled = enabled; }
    void Set_lowThreshold(int value) { m_paramLowThreshold = value; }
    int  Get_lowThreshold() { return m_paramLowThreshold; }
    void Set_highThreshold(int value) { m_paramHighThreshold = value; }
    int  Get_highThreshold() { return m_paramHighThreshold; }

    void Set_minHessian(int value);
    int  Get_minHessian();
    int  Get_minHessianPhysical();
    void Set_maxFeatures(int value)
    {
        if (m_paramMaxFeatures != value)
        {
            m_paramMaxFeatures = value;
            m_surfaceDetectionParamsChanging = true;
        }
    }
    int  Get_maxFeatures() { return m_paramMaxFeatures; }

    void ShowVisualElements(bool state);
    bool VisualElementsEnabled() { return m_showVisualElements; }
    void SetShowFeaturesButtonState(bool state) { m_paramShowElementsButtonState = state; }
    bool GetShowFeaturesButtonState() { return m_paramShowElementsButtonState; }
    void SetNoiseFilterState(bool enable) { m_paramNoiseFilterState = enable; }
    bool GetNoiseFilterState() { return m_paramNoiseFilterState; }

    void SetVideoLogging(bool enable) { m_videoLogEnabled = enable; }
    bool GetVideoLogging() { return m_videoLogEnabled; }

public:
    // Displaying visual aid for solar system object parameter tuning
    bool m_showMinMaxDiameters;
    void RefreshMinMaxDiameters() { m_showMinMaxDiameters = true; }
    void VisualHelper(wxDC& dc, Star primaryStar, double scaleFactor);

private:
    wxStopWatch m_SolarSystemObjWatchdog;
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

private:
    void    SaveVideoFrame(cv::Mat& FullFrame, cv::Mat& img8, bool roiActive, int bppFactor);
    double  ComputeSobelSharpness(const cv::Mat& img);
    double  CalcSharpness(cv::Mat& FullFrame, cv::Point2f& clickedPoint, bool detectionResult);
    void    CalcLineParams(CircleDescriptor p1, CircleDescriptor p2);
    int     RefineDiskCenter(float& bestScore, CircleDescriptor& diskCenter, std::vector<cv::Point2f>& diskContour, int minRadius, int maxRadius, float searchRadius, float resolution = 1.0);
    float   FindContourCenter(CircleDescriptor& diskCenter, CircleDescriptor& smallestCircle, std::vector<cv::Point2f>& bestContourVector, cv::Moments& mu, int minRadius, int maxRadius);
    void    FindCenters(cv::Mat image, const std::vector<cv::Point>& contour, CircleDescriptor& bestCentroid, CircleDescriptor& smallestCircle, std::vector<cv::Point2f>& bestContour, cv::Moments& mu, int minRadius, int maxRadius);
    bool    FindOrbisCenter(cv::Mat img8, int minRadius, int maxRadius, bool roiActive, cv::Point2f& clickedPoint, cv::Rect& roiRect, bool activeRoiLimits, float distanceRoiMax);

    cv::Point2f calculateCentroid(const std::vector<cv::KeyPoint>& keypoints, cv::Point2f& clickedPoint);
    bool areCollinear(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::KeyPoint& kp3);
    bool validateAndFilterKeypoints(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::KeyPoint>& filteredKeypoints);
    bool DetectSurfaceFeatures(cv::Mat image, cv::Point2f& clickedPoint, bool autoSelect);
    void UpdateDetectionErrorInSimulator(cv::Point2f& clickedPoint);
};
