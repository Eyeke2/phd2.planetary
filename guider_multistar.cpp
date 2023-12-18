/*
 *  guider_multistar.cpp
 *  PHD Guiding

 *  Original guider_onestar Created by Craig Stark.
 *  Copyright (c) 2006-2010 Craig Stark.
 *  All rights reserved.
 *
 *  guider_onestar completely refactored by Bret McKee
 *  Copyright (c) 2012 Bret McKee
 *  All rights reserved.
 *
 *  guider_multistar extensions created by Bruce Waddington
 *  Copyright (c) 2020 Bruce Waddington
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

class MassChecker
{
    enum { DefaultTimeWindowMs = 22500 };

    struct Entry
    {
        wxLongLong_t time;
        double mass;
    };

    std::deque<Entry> m_data;
    double m_highMass;   // high-water mark
    double m_lowMass;    // low-water mark
    unsigned long m_timeWindow;
    double *m_tmp;
    size_t m_tmpSize;
    int m_exposure;
    bool m_isAutoExposure;

public:

    MassChecker()
        : m_highMass(0.),
          m_lowMass(9e99),
          m_tmp(0),
          m_tmpSize(0),
          m_exposure(0),
          m_isAutoExposure(false)
    {
        SetTimeWindow(DefaultTimeWindowMs);
    }

    ~MassChecker()
    {
        delete[] m_tmp;
    }

    void SetTimeWindow(unsigned int milliseconds)
    {
        // an abrupt change in mass will affect the median after approx m_timeWindow/2
        m_timeWindow = milliseconds * 2;
    }

    void SetExposure(int exposure, bool isAutoExp)
    {
        if (isAutoExp != m_isAutoExposure)
        {
            m_isAutoExposure = isAutoExp;
            m_exposure = exposure;
            Reset();
        }
        else if (exposure != m_exposure)
        {
            m_exposure = exposure;
            if (!m_isAutoExposure)
            {
                Reset();
            }
        }
    }

    double AdjustedMass(double mass) const
    {
        return m_isAutoExposure ? mass / (double) m_exposure : mass;
    }

    void AppendData(double mass)
    {
        wxLongLong_t now = ::wxGetUTCTimeMillis().GetValue();
        wxLongLong_t oldest = now - m_timeWindow;

        while (m_data.size() > 0 && m_data.front().time < oldest)
            m_data.pop_front();

        Entry entry;
        entry.time = now;
        entry.mass = AdjustedMass(mass);
        m_data.push_back(entry);
    }

    bool CheckMass(double mass, double threshold, double limits[4])
    {
        if (m_data.size() < 5)
            return false;

        if (m_tmpSize < m_data.size())
        {
            delete[] m_tmp;
            m_tmpSize = m_data.size() + 10;
            m_tmp = new double[m_tmpSize];
        }

        std::deque<Entry>::const_iterator it = m_data.begin();
        std::deque<Entry>::const_iterator end = m_data.end();
        double *p = &m_tmp[0];
        for (; it != end; ++it)
            *p++ = it->mass;

        size_t mid = m_data.size() / 2;
        std::nth_element(&m_tmp[0], &m_tmp[mid], &m_tmp[m_data.size()]);
        double med = m_tmp[mid];

        if (med > m_highMass)
            m_highMass = med;
        if (med < m_lowMass)
            m_lowMass = med;

        // let the low water mark drift to follow the median so that it moves back up after a
        // period of intermittent clouds has brought it down
        m_lowMass += .05 * (med - m_lowMass);

        limits[0] = m_lowMass * (1. - threshold);
        limits[1] = med;
        limits[2] = m_highMass * (1. + threshold);
        // when mass is depressed by sky conditions, we still want to trigger a rejection when
        // there is a large spike in mass, even if it is still below the high water mark-based
        // threhold
        limits[3] = med * (1. + 2.0 * threshold);

        double adjmass = AdjustedMass(mass);
        bool reject = adjmass < limits[0] || adjmass > limits[2] || adjmass > limits[3];

        if (reject && m_isAutoExposure)
        {
            // convert back to mass-like numbers for logging by caller
            for (int i = 0; i < 4; i++)
                limits[i] *= (double) m_exposure;
        }

        return reject;
    }

    void Reset()
    {
        m_data.clear();
        m_highMass = 0.;
        m_lowMass = 9e99;
    }
};

static const double DefaultMassChangeThreshold = 0.5;

enum {
    MIN_SEARCH_REGION = 7,
    DEFAULT_SEARCH_REGION = 15,
    MAX_SEARCH_REGION = 50,
    DEFAULT_MAX_STAR_COUNT = 9,
    DEFAULT_STABILITY_SIGMAX = 5,
    MAX_LIST_SIZE = 12
};

BEGIN_EVENT_TABLE(GuiderMultiStar, Guider)
    EVT_PAINT(GuiderMultiStar::OnPaint)
    EVT_LEFT_DOWN(GuiderMultiStar::OnLClick)
END_EVENT_TABLE()

// Define a constructor for the guide canvas
GuiderMultiStar::GuiderMultiStar(wxWindow *parent)
    : Guider(parent, XWinSize, YWinSize),
      m_massChecker(new MassChecker()),
      m_stabilizing(false), m_multiStarMode(true), m_lastPrimaryDistance(0),
      m_lockPositionMoved(false),
      m_maxStars(DEFAULT_MAX_STAR_COUNT),
      m_stabilitySigmaX(DEFAULT_STABILITY_SIGMAX),
      m_lastStarsUsed(0)
{
    SetState(STATE_UNINITIALIZED);
    m_primaryDistStats = new DescriptiveStats();
    m_PlanetWatchdog.Start();
}

GuiderMultiStar::~GuiderMultiStar()
{
    delete m_massChecker;
    delete m_primaryDistStats;
    delete m_Planet.eclipse_edges;
}

void GuiderMultiStar::SetMultiStarMode(bool val)
{
    bool oldVal = m_multiStarMode;
    bool autoFindForced = false;
    m_multiStarMode = val;
    if (val && !oldVal)
    {
        m_primaryDistStats->ClearAll();
        if (GetState() >= STATE_SELECTED)           // If we have a single star, need to force an auto-find to be sure we have the right secondary stars
        {
            StopGuiding();
            InvalidateCurrentPosition(true);
            if (!AutoSelect(wxRect(0, 0, 0, 0)))
            {
                StartGuiding();
                autoFindForced = true;
            }
        }
    }
    if (!val)
        m_stabilizing = false;
    pConfig->Profile.SetBoolean("/guider/multistar/enabled", m_multiStarMode);
    wxString msg = wxString::Format("MultiStar mode %s", (val ? "enabled" : "disabled"));
    if (autoFindForced)
        msg += ", AutoFind forced\n";
    else
        msg += "\n";
    Debug.Write(msg);
    pFrame->NotifyGuidingParam("MultiStar", m_multiStarMode ? "true" : "false", true);
}

void GuiderMultiStar::ClearSecondaryStars()
{
    if (m_guideStars.size() > 1)
    {
        m_guideStars.erase(m_guideStars.begin() + 1, m_guideStars.end());
        Debug.Write("MultiStar: secondary guide stars cleared");
    }
}
void GuiderMultiStar::LoadProfileSettings()
{
    Guider::LoadProfileSettings();

    double massChangeThreshold = pConfig->Profile.GetDouble("/guider/onestar/MassChangeThreshold",
            DefaultMassChangeThreshold);
    SetMassChangeThreshold(massChangeThreshold);

    bool massChangeThreshEnabled = pConfig->Profile.GetBoolean("/guider/onestar/MassChangeThresholdEnabled", massChangeThreshold != 1.0);
    SetMassChangeThresholdEnabled(massChangeThreshEnabled);

    bool tolerateJumps = pConfig->Profile.GetBoolean("/guider/onestar/TolerateJumpsEnabled", false);
    double tolerateJumpsThresh = pConfig->Profile.GetDouble("/guider/onestar/TolerateJumpsThreshold", 4.0);
    SetTolerateJumps(tolerateJumps, tolerateJumpsThresh);

    int searchRegion = pConfig->Profile.GetInt("/guider/onestar/SearchRegion", DEFAULT_SEARCH_REGION);
    SetSearchRegion(searchRegion);

    SetMultiStarMode(pConfig->Profile.GetBoolean("/guider/multistar/enabled", false));
}

bool GuiderMultiStar::GetMassChangeThresholdEnabled() const
{
    return m_massChangeThresholdEnabled;
}

void GuiderMultiStar::SetMassChangeThresholdEnabled(bool enable)
{
    m_massChangeThresholdEnabled = enable;
    pConfig->Profile.SetBoolean("/guider/onestar/MassChangeThresholdEnabled", enable);
}

double GuiderMultiStar::GetMassChangeThreshold() const
{
    return m_massChangeThreshold;
}

bool GuiderMultiStar::SetMassChangeThreshold(double massChangeThreshold)
{
    bool bError = false;

    try
    {
        if (massChangeThreshold < 0.0)
        {
            throw ERROR_INFO("massChangeThreshold < 0");
        }

        m_massChangeThreshold = massChangeThreshold;
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);

        bError = true;
        m_massChangeThreshold = DefaultMassChangeThreshold;
    }

    pConfig->Profile.SetDouble("/guider/onestar/MassChangeThreshold", m_massChangeThreshold);

    return bError;
}

bool GuiderMultiStar::SetTolerateJumps(bool enable, double threshold)
{
    m_tolerateJumpsEnabled = enable;
    pConfig->Profile.SetBoolean("/guider/onestar/TolerateJumpsEnabled", enable);

    m_tolerateJumpsThreshold = threshold;
    pConfig->Profile.SetDouble("/guider/onestar/TolerateJumpsThreshold", threshold);

    return false;
}

bool GuiderMultiStar::SetSearchRegion(int searchRegion)
{
    bool bError = false;

    try
    {
        if (searchRegion < MIN_SEARCH_REGION)
        {
            m_searchRegion = MIN_SEARCH_REGION;
            throw ERROR_INFO("searchRegion < MIN_SEARCH_REGION");
        }
        else if (searchRegion > MAX_SEARCH_REGION)
        {
            m_searchRegion = MAX_SEARCH_REGION;
            throw ERROR_INFO("searchRegion > MAX_SEARCH_REGION");
        }
        m_searchRegion = searchRegion;
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);
        bError = true;
    }

    pConfig->Profile.SetInt("/guider/onestar/SearchRegion", m_searchRegion);

    return bError;
}

bool GuiderMultiStar::SetCurrentPosition(const usImage *pImage, const PHD_Point& position)
{
    bool bError = true;

    try
    {
        if (!position.IsValid())
        {
            throw ERROR_INFO("position is invalid");
        }

        double x = position.X;
        double y = position.Y;

        Debug.Write(wxString::Format("SetCurrentPosition(%.2f,%.2f)\n", x, y ));

        if ((x <= 0) || (x >= pImage->Size.x))
        {
            throw ERROR_INFO("invalid x value");
        }

        if ((y <= 0) || (y >= pImage->Size.y))
        {
            throw ERROR_INFO("invalid y value");
        }

        m_massChecker->Reset();
        int searchRegion = (pFrame->GetStarFindMode() == Star::FIND_PLANET) ? m_Planet.radius : m_searchRegion;
        bError = !m_primaryStar.Find(pImage, searchRegion, x, y, pFrame->GetStarFindMode(),
                              GetMinStarHFD(), GetMaxStarHFD(), pCamera->GetSaturationADU(), Star::FIND_LOGGING_VERBOSE);
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);
    }

    return bError;
}

static wxString StarStatusStr(const Star& star)
{
    if (!star.IsValid())
        return _("No star selected");

    switch (star.GetError())
    {
    case Star::STAR_LOWSNR:        return _("Star lost - low SNR");
    case Star::STAR_LOWMASS:       return _("Star lost - low mass");
    case Star::STAR_LOWHFD:        return _("Star lost - low HFD");
    case Star::STAR_TOO_NEAR_EDGE: return _("Star too near edge");
    case Star::STAR_MASSCHANGE:    return _("Star lost - mass changed");
    default:                       return _("No star found");
    }
}

static wxString StarStatus(const Star& star)
{
    wxString status = wxString::Format(_("m=%.0f SNR=%.1f"), star.Mass, star.SNR);

    if (star.GetError() == Star::STAR_SATURATED)
        status += _T(" ") + _("Saturated");

    int exp;
    bool auto_exp;
    pFrame->GetExposureInfo(&exp, &auto_exp);

    if (auto_exp)
    {
        status += _T(" ");
        if (exp >= 1)
            status += wxString::Format(_("Exp=%0.1f s"), (double) exp / 1000.);
        else
            status += wxString::Format(_("Exp=%d ms"), exp);
    }

    return status;
}

bool GuiderMultiStar::AutoSelect(const wxRect& roi)
{
    Debug.Write("GuiderMultiStar::AutoSelect enter\n");

    bool error = false;

    usImage *image = CurrentImage();

    try
    {
        if (!image || !image->ImageData)
        {
            throw ERROR_INFO("No Current Image");
        }

        // If mount is not calibrated, we need to chose a star a bit farther
        // from the egde to allow for the motion of the star during
        // calibration
        //
        int edgeAllowance = 0;
        if (pMount && pMount->IsConnected() && !pMount->IsCalibrated())
            edgeAllowance = wxMax(edgeAllowance, pMount->CalibrationTotDistance());
        if (pSecondaryMount && pSecondaryMount->IsConnected() && !pSecondaryMount->IsCalibrated())
            edgeAllowance = wxMax(edgeAllowance, pSecondaryMount->CalibrationTotDistance());

        GuideStar newStar;
        int searchRegion;
        Star::FindMode findMode = pFrame->GetStarFindMode();

        if (findMode == Star::FIND_PLANET)
        {
            if (FindPlanet(image, true))
            {
                newStar.X = newStar.referencePoint.X = m_Planet.center_x;
                newStar.Y = newStar.referencePoint.Y = m_Planet.center_y;
                searchRegion = m_Planet.radius;
                m_guideStars.clear();
                m_guideStars.push_back(newStar);
            } else
            {
                throw ERROR_INFO("Unable to AutoFind");
            }
        } else
        {
            searchRegion = m_searchRegion;
            findMode = Star::FIND_CENTROID;
            if (!newStar.AutoFind(*image, edgeAllowance, m_searchRegion, roi, m_guideStars, MAX_LIST_SIZE))
            {
                throw ERROR_INFO("Unable to AutoFind");
            }
        }

        m_massChecker->Reset();

        if (!m_primaryStar.Find(image, searchRegion, newStar.X, newStar.Y, findMode, GetMinStarHFD(), GetMaxStarHFD(),
                         pCamera->GetSaturationADU(), Star::FIND_LOGGING_VERBOSE))
        {
            throw ERROR_INFO("Unable to find");
        }

        // DEBUG OUTPUT
        wxString buff = wxString::Format("MultiStar: List (%d): ", m_guideStars.size());
        for (auto pGS = m_guideStars.begin(); pGS != m_guideStars.end(); ++pGS)
        {
            buff += wxString::Format("{%0.2f, %0.2f}(%0.1f), ", pGS->X, pGS->Y, pGS->SNR);
        }
        Debug.Write(buff + "\n");

        m_primaryDistStats->ClearAll();

        if (SetLockPosition(m_primaryStar))
        {
            throw ERROR_INFO("Unable to set Lock Position");
        }

        if (GetState() == STATE_SELECTING)
        {
            // immediately advance the state machine now, rather than waiting for
            // the next exposure to complete. Socket server clients are going to
            // try to start guiding after selecting the star, but guiding will fail
            // to start if state is still STATE_SELECTING
            Debug.Write(wxString::Format("AutoSelect: state = %d, call UpdateGuideState\n", GetState()));
            UpdateGuideState(NULL, false);
        }

        UpdateImageDisplay();

        pFrame->StatusMsg(wxString::Format(_("Auto-selected star at (%.1f, %.1f)"), m_primaryStar.X, m_primaryStar.Y));
        pFrame->UpdateStatusBarStarInfo(m_primaryStar.SNR, m_primaryStar.GetError() == Star::STAR_SATURATED);
        pFrame->pProfile->UpdateData(image, m_primaryStar.X, m_primaryStar.Y);
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);
        error = true;
    }

    if (image && image->ImageData)
    {
        if (error)
            Debug.Write("GuiderMultiStar::AutoSelect failed.\n");

        ImageLogger::LogAutoSelectImage(image, !error);
    }

    return error;
}

inline static wxRect SubframeRect(const PHD_Point& pos, int halfwidth)
{
    return wxRect(ROUND(pos.X) - halfwidth,
                  ROUND(pos.Y) - halfwidth,
                  2 * halfwidth + 1,
                  2 * halfwidth + 1);
}

wxRect GuiderMultiStar::GetBoundingBox() const
{
    enum { SUBFRAME_BOUNDARY_PX = 0 };

    GUIDER_STATE state = GetState();

    bool subframe;
    PHD_Point pos;

    switch (state) {
    case STATE_SELECTED:
    case STATE_CALIBRATING_PRIMARY:
    case STATE_CALIBRATING_SECONDARY:
        subframe = m_primaryStar.WasFound();
        pos = CurrentPosition();
        break;
    case STATE_GUIDING: {
        subframe = m_primaryStar.WasFound();  // true;
        // As long as the star is close to the lock position, keep the subframe
        // at the lock position. Otherwise, follow the star.
        double dist = CurrentPosition().Distance(LockPosition());
        if ((int) dist > m_searchRegion / 3)
            pos = CurrentPosition();
        else
            pos = LockPosition();
        break;
    }
    default:
        subframe = false;
    }

    if (m_forceFullFrame)
    {
        subframe = false;
    }

    if (subframe)
    {
        wxRect box(SubframeRect(pos, m_searchRegion + SUBFRAME_BOUNDARY_PX));
        box.Intersect(wxRect(pCamera->FullSize));
        return box;
    }
    else
    {
        return wxRect(0, 0, 0, 0);
    }
}

void GuiderMultiStar::InvalidateCurrentPosition(bool fullReset)
{
    m_primaryStar.Invalidate();

    if (fullReset)
    {
        m_primaryStar.X = m_primaryStar.Y = 0.0;
    }
}

struct DistanceChecker
{
    enum State
    {
        ST_GUIDING,
        ST_WAITING,
        ST_RECOVERING,
    };
    State m_state;
    wxLongLong_t m_expires;
    double m_forceTolerance;

    enum { WAIT_INTERVAL_MS = 5000 };

    DistanceChecker() : m_state(ST_GUIDING) { }

    void Activate()
    {
        if (m_state == ST_GUIDING)
        {
            Debug.Write("DistanceChecker: activated\n");
            m_state = ST_WAITING;
            m_expires = ::wxGetUTCTimeMillis().GetValue() + WAIT_INTERVAL_MS;
            m_forceTolerance = 2.0;
        }
    }

    static bool _CheckDistance(double distance, bool raOnly, double tolerance)
    {
        enum { MIN_FRAMES_FOR_STATS = 10 };
        Guider *guider = pFrame->pGuider;
        if (!guider->IsGuiding() || guider->IsPaused() || PhdController::IsSettling() ||
            guider->CurrentErrorFrameCount() < MIN_FRAMES_FOR_STATS)
        {
            return true;
        }
        double avgDist = guider->CurrentErrorSmoothed(raOnly);
        double threshold = tolerance * avgDist;
        if (distance > threshold)
        {
            Debug.Write(wxString::Format("DistanceChecker: reject for large offset (%.2f > %.2f) avgDist = %.2f count = %u\n",
                                         distance, threshold, avgDist, guider->CurrentErrorFrameCount()));
            return false;
        }
        return true;
    }

    bool CheckDistance(double distance, bool raOnly, double tolerance)
    {
        if (m_forceTolerance != 0.)
            tolerance = m_forceTolerance;

        bool small_offset = _CheckDistance(distance, raOnly, tolerance);

        switch (m_state)
        {
        default:
        case ST_GUIDING:
            if (small_offset)
                return true;

            Debug.Write("DistanceChecker: activated\n");
            m_state = ST_WAITING;
            m_expires = ::wxGetUTCTimeMillis().GetValue() + WAIT_INTERVAL_MS;
            return false;

        case ST_WAITING:
        {
            if (small_offset)
            {
                Debug.Write("DistanceChecker: deactivated\n");
                m_state = ST_GUIDING;
                m_forceTolerance = 0.;
                return true;
            }
            // large distance
            wxLongLong_t now = ::wxGetUTCTimeMillis().GetValue();
            if (now < m_expires)
            {
                // reject frame
                return false;
            }
            // timed-out
            Debug.Write("DistanceChecker: begin recovering\n");
            m_state = ST_RECOVERING;
            // fall through
        }

        case ST_RECOVERING:
            if (small_offset)
            {
                Debug.Write("DistanceChecker: deactivated\n");
                m_state = ST_GUIDING;
            }
            return true;
        }
    }
};

wxString GuiderMultiStar::GetStarCount() const
{
    // no weird displays if stars are being removed from list
    return wxString::Format("%u/%u", wxMin(m_starsUsed, static_cast<unsigned int>(m_guideStars.size())),
                            static_cast<unsigned int>(m_guideStars.size()));
}

// Private method to build compact logging string for how secondary stars were used
static void AppendStarUse(wxString& secondaryInfo, int starNum, double dX, double dY, double weight, const wxString& flag)
{
    secondaryInfo += wxString::Format("[#%d %0.2f,%0.2f,%0.2f,%s] ", starNum, dX, dY, weight, flag);
}

// Use secondary stars to refine Offset value if appropriate.  Return of true means offset has been adjusted
bool GuiderMultiStar::RefineOffset(const usImage *pImage, GuiderOffset *pOffset)
{
    double primaryDistance;
    double secondaryDistance;
    double primarySigma = 0;
    bool averaged = false;
    int validStars = 0;
    GuiderOffset origOffset = *pOffset;
    m_starsUsed = 1;
    bool erasures = false;
    bool refined = false;

    // Primary star is in position 0 of the list
    try
    {
        if (IsGuiding() && m_guideStars.size() > 1 && pMount->GetGuidingEnabled() && !PhdController::IsSettling())
        {
            double sumWeights = 1;
            double sumX = origOffset.cameraOfs.X;
            double sumY = origOffset.cameraOfs.Y;
            primaryDistance = hypot(sumX, sumY);

            m_primaryDistStats->AddValue(primaryDistance);

#define Iter_Inx(p) (p - m_guideStars.begin())

            if (m_primaryDistStats->GetCount() > 5)
            {
                primarySigma = m_primaryDistStats->GetSigma();
                if (!m_stabilizing && primaryDistance > m_stabilitySigmaX * primarySigma)
                {
                    m_stabilizing = true;
                    Debug.Write("MultiStar: large primary error, entering stabilization period\n");
                }
                else if (m_stabilizing)
                {
                    if (primaryDistance <= 2 * primarySigma)
                    {
                        m_stabilizing = false;
                        Debug.Write("MultiStar: exiting stabilization period\n");
                        if (m_lockPositionMoved)
                        {
                            m_lockPositionMoved = false;
                            Debug.Write("MultiStar: updating star positions after lock position change\n");
                            for (auto pGS = m_guideStars.begin() + 1; pGS != m_guideStars.end();)
                            {
                                PHD_Point expectedLoc = m_primaryStar + pGS->offsetFromPrimary;
                                bool found;
                                if (IsValidSecondaryStarPosition (expectedLoc))
                                    found = pGS->Find(pImage, m_searchRegion, expectedLoc.X, expectedLoc.Y, pFrame->GetStarFindMode(),
                                        GetMinStarHFD(), GetMaxStarHFD(), pCamera->GetSaturationADU(), Star::FIND_LOGGING_VERBOSE);
                                else
                                    found = pGS->Find(pImage, m_searchRegion, pGS->X, pGS->Y, pFrame->GetStarFindMode(),
                                        GetMinStarHFD(), GetMaxStarHFD(), pCamera->GetSaturationADU(), Star::FIND_LOGGING_VERBOSE);
                                if (found)
                                {
                                    pGS->referencePoint.X = pGS->X;
                                    pGS->referencePoint.Y = pGS->Y;
                                    pGS->wasLost = false;
                                    ++pGS;
                                }
                                else
                                {
                                    // Don't need to update reference point, lost star will continue to use the offsetFromPrimary location for possible recovery
                                    pGS->wasLost = true;
                                    ++pGS;
                                }
                            }
                            return false;                 // All the secondary stars reference points reflect current positions
                        }
                    }
                }
            }
            else
                m_stabilizing = true;                       // get some data for primary star movement

            if (!m_stabilizing && m_guideStars.size() > 1 && (sumX != 0 || sumY != 0))
            {
                wxString secondaryInfo = "MultiStar: ";
                for (auto pGS = m_guideStars.begin() + 1; pGS != m_guideStars.end();)
                {
                    if (m_starsUsed >= m_maxStars || m_guideStars.size() == 1)
                        break;
                    bool found = false;
                    if (pGS->wasLost)
                    {
                        // Look for it based on its original offset from the primary star
                        PHD_Point expectedLoc = m_primaryStar + pGS->offsetFromPrimary;
                        found = pGS->Find(pImage, m_searchRegion, expectedLoc.X, expectedLoc.Y, pFrame->GetStarFindMode(),
                            GetMinStarHFD(), GetMaxStarHFD(), pCamera->GetSaturationADU(), Star::FIND_LOGGING_MINIMAL);
                    }
                    else
                        // Look for it where we last found it
                        found = pGS->Find(pImage, m_searchRegion, pGS->X, pGS->Y, pFrame->GetStarFindMode(),
                            GetMinStarHFD(), GetMaxStarHFD(), pCamera->GetSaturationADU(), Star::FIND_LOGGING_MINIMAL);
                    if (found)
                    {
                        double dX = pGS->X - pGS->referencePoint.X;
                        double dY = pGS->Y - pGS->referencePoint.Y;

                        pGS->wasLost = false;
                        m_starsUsed++;

                        if (dX != 0. || dY != 0.)
                        {
                            // Handle zero-counting - suspect results of exactly zero movement
                            if (dX == 0. || dY == 0.)
                                ++pGS->zeroCount;
                            else if (pGS->zeroCount > 0)
                                --pGS->zeroCount;

                            if (pGS->zeroCount == 5)
                            {
                                AppendStarUse(secondaryInfo, Iter_Inx(pGS), 0, 0, 0, "DZ");
                                pGS = m_guideStars.erase(pGS);
                                erasures = true;
                                continue;
                            }

                            // Handle suspicious excursions - counted as "misses"
                            secondaryDistance = hypot(dX, dY);
                            if (secondaryDistance > 2.5 * primarySigma)
                            {
                                if (++pGS->missCount > 10)
                                {
                                    // Reset the reference point to wherever it is now
                                    pGS->referencePoint.X = pGS->X;
                                    pGS->referencePoint.Y = pGS->Y;
                                    pGS->missCount = 0;
                                    AppendStarUse(secondaryInfo, Iter_Inx(pGS), dX, dY, 0, "R");
                                }
                                else
                                    AppendStarUse(secondaryInfo, Iter_Inx(pGS), dX, dY, 0, "M" + std::to_string(pGS->missCount));
                                ++pGS;
                                continue;
                            }
                            else if (pGS->missCount > 0)
                            {
                                --pGS->missCount;
                            }

                            // At this point we have usable data from the secondary star
                            double wt = (pGS->SNR / m_primaryStar.SNR);
                            sumX += wt * dX;
                            sumY += wt * dY;
                            sumWeights += wt;
                            averaged = true;
                            validStars++;

                            AppendStarUse(secondaryInfo, Iter_Inx(pGS), dX, dY, wt, "U");
                        }
                        else                                          // exactly zero on both axes, probably a hot pixel, drop it
                        {
                            AppendStarUse(secondaryInfo, Iter_Inx(pGS), 0, 0, 0, "DZ");
                            pGS = m_guideStars.erase(pGS);
                            erasures = true;
                        }
                    }
                    else
                    {
                        // star not found in its search region
                        AppendStarUse(secondaryInfo, Iter_Inx(pGS), 0, 0, 0, "L");
                        pGS->wasLost = true;
                    }
                    if (!erasures)
                        ++pGS;
                    else
                        erasures = false;
                }                                   // End of looping through secondary stars
                Debug.Write(secondaryInfo + "\n");

                if (averaged)
                {
                    sumX = sumX / sumWeights;
                    sumY = sumY / sumWeights;
                    if (hypot(sumX, sumY) < primaryDistance)                                   // Apply average only if its smaller than single-star delta
                    {
                        pOffset->cameraOfs.X = sumX;
                        pOffset->cameraOfs.Y = sumY;
                        refined = true;
                    }
                    Debug.Write(wxString::Format("%s, %d included, MultiStar: {%0.2f, %0.2f}, one-star: {%0.2f, %0.2f}\n", (refined ? "refined" : "single-star"),
                        validStars, sumX, sumY,
                        origOffset.cameraOfs.X, origOffset.cameraOfs.Y));
                }
            }
        }
    }
    catch (const wxString& msg)
    {
        Debug.Write(wxString::Format("MultiStar fault: exception at %d, %s, reverting to single-star mode\n", __LINE__, msg));
        m_multiStarMode = false;
    }

    return refined;
#undef Iter_Inx
}

static DistanceChecker s_distanceChecker;

bool GuiderMultiStar::UpdateCurrentPosition(const usImage *pImage, GuiderOffset *ofs, FrameDroppedInfo *errorInfo)
{
    if (!m_primaryStar.IsValid() && m_primaryStar.X == 0.0 && m_primaryStar.Y == 0.0)
    {
        Debug.Write("UpdateCurrentPosition: no star selected\n");
        errorInfo->starError = Star::STAR_ERROR;
        errorInfo->starMass = 0.0;
        errorInfo->starSNR = 0.0;
        errorInfo->starHFD = 0.0;
        errorInfo->status = _("No star selected");
        ImageLogger::LogImageStarDeselected(pImage);
        return true;
    }

    bool bError = false;

    try
    {
        Star newStar(m_primaryStar);
        int search_region = m_searchRegion;

        if (pFrame->GetStarFindMode() == Star::FIND_PLANET)
        {
            if (!FindPlanet(pImage))
            {
                errorInfo->starError = newStar.GetError();
                errorInfo->starMass = 0.0;
                errorInfo->starSNR = 0.0;
                errorInfo->starHFD = 0.0;
                errorInfo->status = _("Object not found");
                throw ERROR_INFO("UpdateCurrentPosition():newStar not found");
            }
            double newpos_x = m_Planet.center_x;
            double newpos_y = m_Planet.center_y;
            newStar.SetXY(newpos_x, newpos_y);
            search_region = m_Planet.radius;
        }

        if (!newStar.Find(pImage, search_region, pFrame->GetStarFindMode(), GetMinStarHFD(),
            GetMaxStarHFD(), pCamera->GetSaturationADU(), Star::FIND_LOGGING_VERBOSE))
        {
            errorInfo->starError = newStar.GetError();
            errorInfo->starMass = 0.0;
            errorInfo->starSNR = 0.0;
            errorInfo->starHFD = 0.0;
            errorInfo->status = StarStatusStr(newStar);
            m_primaryStar.SetError(newStar.GetError());

            s_distanceChecker.Activate();
            ImageLogger::LogImage(pImage, *errorInfo);

            throw ERROR_INFO("UpdateCurrentPosition():newStar not found");
        }

        // check to see if it seems like the star we just found was the
        // same as the original star by comparing the mass
        if (m_massChangeThresholdEnabled)
        {
            int exposure;
            bool isAutoExp;
            pFrame->GetExposureInfo(&exposure, &isAutoExp);
            m_massChecker->SetExposure(exposure, isAutoExp);
            double limits[4];
            if (m_massChecker->CheckMass(newStar.Mass, m_massChangeThreshold, limits))
            {
                m_primaryStar.SetError(Star::STAR_MASSCHANGE);
                errorInfo->starError = Star::STAR_MASSCHANGE;
                errorInfo->starMass = newStar.Mass;
                errorInfo->starSNR = newStar.SNR;
                errorInfo->starHFD = newStar.HFD;
                errorInfo->status = StarStatusStr(m_primaryStar);
                pFrame->StatusMsg(wxString::Format(_("Mass: %.f vs %.f"), newStar.Mass, limits[1]));

                Debug.Write(wxString::Format("UpdateCurrentPosition: star mass new=%.1f exp=%.1f thresh=%.0f%% limits=(%.1f, %.1f, %.1f)\n",
                    newStar.Mass, limits[1], m_massChangeThreshold * 100., limits[0], limits[2], limits[3]));

                m_massChecker->AppendData(newStar.Mass);

                s_distanceChecker.Activate();
                ImageLogger::LogImage(pImage, *errorInfo);

                throw THROW_INFO("massChangeThreshold error");
            }
        }

        const PHD_Point& lockPos = LockPosition();
        double distance;
        bool raOnly = MyFrame::GuidingRAOnly();
        if (lockPos.IsValid())
        {
            if (raOnly)
                distance = fabs(newStar.X - lockPos.X);
            else
                distance = newStar.Distance(lockPos);
        }
        else
            distance = 0.;

        double tolerance = m_tolerateJumpsEnabled ? m_tolerateJumpsThreshold : 9e99;

        if (!s_distanceChecker.CheckDistance(distance, raOnly, tolerance))
        {
            m_primaryStar.SetError(Star::STAR_ERROR);
            errorInfo->starError = Star::STAR_ERROR;
            errorInfo->starMass = newStar.Mass;
            errorInfo->starSNR = newStar.SNR;
            errorInfo->starHFD = newStar.HFD;
            errorInfo->status = StarStatusStr(m_primaryStar);
            pFrame->StatusMsg(_("Recovering"));

            ImageLogger::LogImage(pImage, *errorInfo);

            throw THROW_INFO("CheckDistance error");
        }

        ImageLogger::LogImage(pImage, distance);

        // update the star position, mass, etc.
        m_primaryStar = newStar;
        m_massChecker->AppendData(newStar.Mass);

        if (lockPos.IsValid())
        {
            ofs->cameraOfs = m_primaryStar - lockPos;
            if (m_multiStarMode && m_guideStars.size() > 1)
            {
                if (RefineOffset(pImage, ofs))
                    distance = hypot(ofs->cameraOfs.X, ofs->cameraOfs.Y);       // Distance is reported to server clients
            }
            else
                m_starsUsed = 1;

            if (pMount && pMount->IsCalibrated())
                pMount->TransformCameraCoordinatesToMountCoordinates(ofs->cameraOfs, ofs->mountOfs, true);
            double distanceRA = ofs->mountOfs.IsValid() ? fabs(ofs->mountOfs.X) : 0.;
            UpdateCurrentDistance(distance, distanceRA);
        }

        pFrame->pProfile->UpdateData(pImage, m_primaryStar.X, m_primaryStar.Y);

        pFrame->AdjustAutoExposure(m_primaryStar.SNR);
        pFrame->UpdateStatusBarStarInfo(m_primaryStar.SNR, m_primaryStar.GetError() == Star::STAR_SATURATED);
        errorInfo->status = StarStatus(m_primaryStar);

        if ((GetState() != STATE_GUIDING) && (pFrame->GetStarFindMode() == Star::FIND_PLANET))
            pFrame->StatusMsg(wxString::Format(_("Object at (%.1f, %.1f) radius=%d"), m_Planet.center_x, m_Planet.center_y, m_Planet.radius));
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);
        bError = true;
        pFrame->ResetAutoExposure(); // use max exposure duration
    }

    return bError;
}

bool GuiderMultiStar::SetLockPosition(const PHD_Point& position)
{
    if (!Guider::SetLockPosition(position))
    {
        if (m_multiStarMode)
        {
            m_lockPositionMoved = true;
            m_stabilizing = true;
            Debug.Write("MultiStar: stabilizing after lock position change\n");
        }
        return false;
    }
    else
        return true;
}

bool GuiderMultiStar::IsValidLockPosition(const PHD_Point& pt)
{
    const usImage *pImage = CurrentImage();
    if (!pImage)
        return false;
    // this is a bit ugly as it is tightly coupled to Star::Find
    return pt.X >= 1 + m_searchRegion &&
        pt.X + 1 + m_searchRegion < pImage->Size.GetX() &&
        pt.Y >= 1 + m_searchRegion &&
        pt.Y + 1 + m_searchRegion < pImage->Size.GetY();
}

bool GuiderMultiStar::IsValidSecondaryStarPosition(const PHD_Point& pt)
{
    const usImage *pImage = CurrentImage();
    if (!pImage)
        return false;
    // As above, tightly coupled to Star::Find but with somewhat relaxed constraints. Find handles cases where search region is only partly within image
    return pt.X >= 5 &&
        pt.X + 5 < pImage->Size.GetX() &&
        pt.Y >= 5 &&
        pt.Y + 5 < pImage->Size.GetY();
}

void GuiderMultiStar::OnLClick(wxMouseEvent &mevent)
{
    try
    {
        if (mevent.GetModifiers() == wxMOD_CONTROL)
        {
            double const scaleFactor = ScaleFactor();
            wxRealPoint pt((double) mevent.m_x / scaleFactor,
                           (double) mevent.m_y / scaleFactor);
            ToggleBookmark(pt);
            m_showBookmarks = true;
            pFrame->bookmarks_menu->Check(MENU_BOOKMARKS_SHOW, GetBookmarksShown());
            Refresh();
            Update();
            return;
        }

        if (GetState() > STATE_SELECTED)
        {
            mevent.Skip();
            throw THROW_INFO("Skipping event because state > STATE_SELECTED");
        }

        if (mevent.GetModifiers() == wxMOD_SHIFT)
        {
            // Deselect guide star
            Debug.Write(wxS("manual deselect\n"));
            InvalidateCurrentPosition(true);
        }
        else
        {
            if ((mevent.m_x <= m_searchRegion) || (mevent.m_x + m_searchRegion >= XWinSize) || (mevent.m_y <= m_searchRegion) || (mevent.m_y + m_searchRegion >= YWinSize))
            {
                mevent.Skip();
                throw THROW_INFO("Skipping event because click outside of search region");
            }

            usImage *pImage = CurrentImage();

            if (pImage->NPixels == 0)
            {
                mevent.Skip();
                throw ERROR_INFO("Skipping event m_pCurrentImage->NPixels == 0");
            }

            double StarX, StarY;

            /* Set ROI to a square around clicked position */
            if (pFrame->GetStarFindMode() == Star::FIND_PLANET)
            {
                m_Planet.clicked_x = mevent.m_x / ScaleFactor();
                m_Planet.clicked_y = mevent.m_y / ScaleFactor();
                m_Planet.clicked = true;

                if (GetRoiEnableState())
                {
                    // Set ROI centered around currently clicked point
                    m_Planet.center_x = m_Planet.clicked_x;
                    m_Planet.center_y = m_Planet.clicked_y;
                    // In no planet was detected earlier use image height as a search diameter
                    if (!m_Planet.detected)
                        m_Planet.roi_radius = std::min(pImage->Size.GetHeight(), pImage->Size.GetWidth()) / 2;
                }

                // Try to locate a planet
                FindPlanet((const usImage*)pImage);
            }

            if ((pFrame->GetStarFindMode() == Star::FIND_PLANET) && m_Planet.detected)
            {
                StarX = (double) m_Planet.center_x;
                StarY = (double) m_Planet.center_y;
            }
            else
            {
                double scaleFactor = ScaleFactor();
                StarX = (double) mevent.m_x / scaleFactor;
                StarY = (double) mevent.m_y / scaleFactor;
            }

            SetCurrentPosition(pImage, PHD_Point(StarX, StarY));

            if (!m_primaryStar.IsValid())
            {
                pFrame->StatusMsg(wxString::Format(_("No star found")));
            }
            else
            {
                SetLockPosition(m_primaryStar);
                if (m_guideStars.size() > 1)
                    ClearSecondaryStars();
                if (m_guideStars.size() == 0)
                {
                    m_guideStars.push_back(m_primaryStar);
                }
                Debug.Write("MultiStar: single-star usage forced by user star selection\n");
                if (pFrame->GetStarFindMode() == Star::FIND_PLANET)
                    pFrame->StatusMsg(wxString::Format(_("Selected point at (%.1f, %.1f)"), m_primaryStar.X, m_primaryStar.Y));
                else
                    pFrame->StatusMsg(wxString::Format(_("Selected star at (%.1f, %.1f)"), m_primaryStar.X, m_primaryStar.Y));
                pFrame->UpdateStatusBarStarInfo(m_primaryStar.SNR, m_primaryStar.GetError() == Star::STAR_SATURATED);
                EvtServer.NotifyStarSelected(CurrentPosition());
                SetState(STATE_SELECTED);
                pFrame->UpdateButtonsStatus();
                pFrame->pProfile->UpdateData(pImage, m_primaryStar.X, m_primaryStar.Y);
            }

            if (pFrame->GetStarFindMode() == Star::FIND_PLANET)
                m_draw_PlanetaryHelper = true;

            Refresh();
            Update();
        }
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);
    }
}

inline static void DrawBox(wxDC& dc, const PHD_Point& star, int halfW, double scale)
{
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    double w = ROUND((halfW * 2 + 1) * scale);
    int xpos = int((star.X - halfW) * scale);
    int ypos = int((star.Y - halfW) * scale);
    if (pFrame->GetStarFindMode() == Star::FIND_PLANET)
    {
        if (pFrame->pGuider->m_Planet.detected)
        {
            int x = int(star.X * scale + 0.5);
            int y = int(star.Y * scale + 0.5);
            int r = int(pFrame->pGuider->m_Planet.radius * scale + 0.5);
            dc.DrawCircle(x, y, r);
            dc.SetPen(wxPen(dc.GetPen().GetColour(), 1, dc.GetPen().GetStyle()));
            dc.DrawRectangle(xpos, ypos, w, w);
        }
    }
    else
    {
        dc.DrawRectangle(xpos, ypos, w, w);
    }
}

// Helper for visualizing planet detection radius
void GuiderMultiStar::PlanetVisualHelper(wxDC &dc)
{
    // Draw the edge bitmap
    if (GetPlanetaryThresholdVisual())
    {
        m_Planet.sync_lock.Lock();
        if (m_Planet.eclipse_edges_valid)
        {
            dc.SetPen(wxPen(wxColour(230, 0, 0), 1, wxPENSTYLE_SOLID));
            for (int y = 0; y < m_Planet.rows; y++)
            {
                for (int x = 0; x < m_Planet.cols; x++)
                {
                    int index = (y * m_Planet.cols + x);
                    if (m_Planet.eclipse_edges[index])
                        dc.DrawPoint((x + m_Planet.offset_x) * m_scaleFactor, (y + m_Planet.offset_y) * m_scaleFactor);
                }
            }
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

// Define the repainting behaviour
void GuiderMultiStar::OnPaint(wxPaintEvent& event)
{
    wxAutoBufferedPaintDC dc(this);
    wxMemoryDC memDC;

    try
    {
        if (PaintHelper(dc, memDC))
        {
            throw ERROR_INFO("PaintHelper failed");
        }
        // PaintHelper drew the image and any overlays
        // now decorate the image to show the selection

        // display bookmarks
        if (m_showBookmarks && m_bookmarks.size() > 0)
        {
            dc.SetPen(wxPen(wxColour(0, 255, 255), 1, wxPENSTYLE_SOLID));
            dc.SetBrush(*wxTRANSPARENT_BRUSH);

            for (std::vector<wxRealPoint>::const_iterator it = m_bookmarks.begin();
                 it != m_bookmarks.end(); ++it)
            {
                wxPoint p((int)(it->x * m_scaleFactor), (int)(it->y * m_scaleFactor));
                dc.DrawCircle(p, 3);
                dc.DrawCircle(p, 6);
                dc.DrawCircle(p, 12);
            }
        }

        // show in-use secondary stars
        if (m_multiStarMode && m_guideStars.size() > 1)
        {
            if (m_primaryStar.WasFound())
                dc.SetPen(wxPen(wxColour(0, 255, 0), 1, wxPENSTYLE_SOLID));
            else
                dc.SetPen(wxPen(wxColour(230, 130, 30), 1, wxPENSTYLE_DOT));
            dc.SetBrush(*wxTRANSPARENT_BRUSH);
            unsigned int starsPlotted = 1;
            if (m_stabilizing)
            {
                if (m_lastStarsUsed == 0)
                    m_lastStarsUsed = wxMin(m_guideStars.size(), (size_t) DEFAULT_MAX_STAR_COUNT);
            }

            for (std::vector<GuideStar>::const_iterator it = m_guideStars.begin() + 1;
                 it != m_guideStars.end(); ++it)
            {
                wxPoint pt((int)(it->referencePoint.X * m_scaleFactor), (int)(it->referencePoint.Y * m_scaleFactor));
                dc.DrawCircle(pt, 6);
                starsPlotted++;
                if (starsPlotted == m_maxStars)
                    break;
            }
            if (!m_stabilizing)
                m_lastStarsUsed = m_starsUsed;
        }

        GUIDER_STATE state = GetState();
        bool FoundStar = m_primaryStar.WasFound();

        int thickness = pFrame->GetStarFindMode() == Star::FIND_PLANET ? 4 : 1;
        if (state == STATE_SELECTED)
        {
            if (FoundStar)
                dc.SetPen(wxPen(wxColour(100,255,90), thickness, wxPENSTYLE_SOLID));  // Draw the box around the star
            else
                dc.SetPen(wxPen(wxColour(230,130,30), thickness, wxPENSTYLE_DOT));
            DrawBox(dc, m_primaryStar, m_searchRegion, m_scaleFactor);
        }
        else if (state == STATE_CALIBRATING_PRIMARY || state == STATE_CALIBRATING_SECONDARY)
        {
            // in the calibration process
            dc.SetPen(wxPen(wxColour(32,196,32), thickness, wxPENSTYLE_SOLID));  // Draw the box around the star
            DrawBox(dc, m_primaryStar, m_searchRegion, m_scaleFactor);
        }
        else if (state == STATE_CALIBRATED || state == STATE_GUIDING)
        {
            // locked and guiding
            if (FoundStar)
                dc.SetPen(wxPen(wxColour(32,196,32), thickness, wxPENSTYLE_SOLID));  // Draw the box around the star
            else
                dc.SetPen(wxPen(wxColour(230,130,30), thickness, wxPENSTYLE_DOT));
            DrawBox(dc, m_primaryStar, m_searchRegion, m_scaleFactor);
        }

        if (GetPlanetaryEnableState() && (m_draw_PlanetaryHelper || GetPlanetaryThresholdVisual()))
            PlanetVisualHelper(dc);
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);
    }
}

void GuiderMultiStar::SaveStarFITS()
{
    double StarX = m_primaryStar.X;
    double StarY = m_primaryStar.Y;
    usImage *pImage = CurrentImage();
    usImage tmpimg;
    wxString imgLogDirectory;

    tmpimg.Init(60,60);
    int start_x = ROUND(StarX)-30;
    int start_y = ROUND(StarY)-30;
    if ((start_x + 60) > pImage->Size.GetWidth())
        start_x = pImage->Size.GetWidth() - 60;
    if ((start_y + 60) > pImage->Size.GetHeight())
        start_y = pImage->Size.GetHeight() - 60;
    int x,y, width;
    width = pImage->Size.GetWidth();
    unsigned short *usptr = tmpimg.ImageData;
    for (y = 0; y < 60; y++)
    {
        for (x = 0; x < 60; x++, usptr++)
            *usptr = *(pImage->ImageData + (y + start_y)*width + (x + start_x));
    }

    imgLogDirectory = Debug.GetLogDir() + PATHSEPSTR + "PHD2_Stars";
    if (!wxDirExists(imgLogDirectory))
        wxFileName::Mkdir(imgLogDirectory, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL);
    wxString fname = imgLogDirectory + PATHSEPSTR + "PHD_GuideStar" + wxDateTime::Now().Format(_T("_%j_%H%M%S")) + ".fit";

    fitsfile *fptr;  // FITS file pointer
    int status = 0;  // CFITSIO status value MUST be initialized to zero!

    PHD_fits_create_file(&fptr, fname, false, &status);

    if (!status)
    {
        long fsize[] = { 60, 60 };
        fits_create_img(fptr, USHORT_IMG, 2, fsize, &status);

        FITSHdrWriter hdr(fptr, &status);

        hdr.write("DATE", wxDateTime::UNow(), wxDateTime::UTC, "file creation time, UTC");
        hdr.write("DATE-OBS", pImage->ImgStartTime, wxDateTime::UTC, "image capture start time, UTC");
        hdr.write("EXPOSURE", (float) pImage->ImgExpDur / 1000.0f, "Exposure time [s]");
        hdr.write("XBINNING", (unsigned int) pCamera->Binning, "Camera X binning");
        hdr.write("YBINNING", (unsigned int) pCamera->Binning, "Camera Y binning");
        hdr.write("XORGSUB", start_x, "Subframe x position in binned pixels");
        hdr.write("YORGSUB", start_y, "Subframe y position in binned pixels");

        if (!status)
        {
            long fpixel[] = { 1, 1, 1 };
            fits_write_pix(fptr, TUSHORT, fpixel, tmpimg.NPixels, tmpimg.ImageData, &status);
        }
    }

    PHD_fits_close_file(fptr);
}

wxString GuiderMultiStar::GetSettingsSummary() const
{
    // return a loggable summary of guider configs
    wxString s = wxString::Format(_T("Search region = %d px, Star mass tolerance "), GetSearchRegion());

    if (GetMassChangeThresholdEnabled())
        s += wxString::Format(_T("= %.1f%%"), GetMassChangeThreshold() * 100.0);
    else
        s += _T("disabled");

    if (m_multiStarMode)
        s += wxString::Format(_T(", Multi-star mode, list size = %d\n "), m_guideStars.size());
    else
        s += ", Single-star mode\n";
    return s;
}

Guider::GuiderConfigDialogPane *GuiderMultiStar::GetConfigDialogPane(wxWindow *pParent)
{
    return new GuiderMultiStarConfigDialogPane(pParent, this);
}

GuiderMultiStar::GuiderMultiStarConfigDialogPane::GuiderMultiStarConfigDialogPane(wxWindow *pParent, GuiderMultiStar *pGuider)
    : GuiderConfigDialogPane(pParent, pGuider)
{

}

void GuiderMultiStar::GuiderMultiStarConfigDialogPane::LayoutControls(Guider *pGuider, BrainCtrlIdMap& CtrlMap)
{
    GuiderConfigDialogPane::LayoutControls(pGuider, CtrlMap);
}

GuiderConfigDialogCtrlSet* GuiderMultiStar::GetConfigDialogCtrlSet(wxWindow *pParent, Guider *pGuider, AdvancedDialog *pAdvancedDialog, BrainCtrlIdMap& CtrlMap)
{
    return new GuiderMultiStarConfigDialogCtrlSet(pParent, pGuider, pAdvancedDialog, CtrlMap);
}

GuiderMultiStarConfigDialogCtrlSet::GuiderMultiStarConfigDialogCtrlSet(wxWindow *pParent, Guider *pGuider, AdvancedDialog *pAdvancedDialog, BrainCtrlIdMap& CtrlMap)
    : GuiderConfigDialogCtrlSet(pParent, pGuider, pAdvancedDialog, CtrlMap)
{
    assert(pGuider);
    m_pGuiderMultiStar = static_cast<GuiderMultiStar *>(pGuider);

    int width;

    width = StringWidth(_T("0000"));
    m_pSearchRegion = pFrame->MakeSpinCtrl(GetParentWindow(AD_szStarTracking), wxID_ANY, _T(" "), wxDefaultPosition,
        wxSize(width, -1), wxSP_ARROW_KEYS, MIN_SEARCH_REGION, MAX_SEARCH_REGION, DEFAULT_SEARCH_REGION, _T("Search"));
    wxSizer *pSearchRegion = MakeLabeledControl(AD_szStarTracking, _("Search region (pixels)"), m_pSearchRegion,
        _("How many pixels (up/down/left/right) do we examine to find the star? Default = 15"));

    wxStaticBoxSizer *pStarMass = new wxStaticBoxSizer(wxHORIZONTAL, GetParentWindow(AD_szStarTracking), _("Star Mass Detection"));
    m_pEnableStarMassChangeThresh = new wxCheckBox(GetParentWindow(AD_szStarTracking), STAR_MASS_ENABLE, _("Enable"));
    m_pEnableStarMassChangeThresh->SetToolTip(_("Check to enable star mass change detection. When enabled, "
        "PHD skips frames when the guide star mass changes by an amount greater than the setting for 'tolerance'."));

    GetParentWindow(AD_szStarTracking)->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &GuiderMultiStarConfigDialogCtrlSet::OnStarMassEnableChecked, this, STAR_MASS_ENABLE);

    width = StringWidth(_T("100.0"));
    m_pMassChangeThreshold = pFrame->MakeSpinCtrlDouble(GetParentWindow(AD_szStarTracking), wxID_ANY, wxEmptyString, wxDefaultPosition,
        wxSize(width, -1), wxSP_ARROW_KEYS, 10.0, 100.0, 0.0, 1.0, _T("MassChangeThreshold"));
    m_pMassChangeThreshold->SetDigits(1);
    wxSizer *pTolerance = MakeLabeledControl(AD_szStarTracking, _("Tolerance"), m_pMassChangeThreshold,
        _("When star mass change detection is enabled, this is the tolerance for star mass changes between frames, in percent. "
        "Larger values are more tolerant (less sensitive) to star mass changes. Valid range is 10-100, default is 50. "
        "If star mass change detection is not enabled then this setting is ignored."));
    pStarMass->Add(m_pEnableStarMassChangeThresh, wxSizerFlags(0).Border(wxTOP, 3));
    pStarMass->Add(pTolerance, wxSizerFlags(0).Border(wxLEFT, 40));

    width = StringWidth(_("65535"));

    double minHFD = pGuider->GetMinStarHFDFloor();
    m_MinHFD = pFrame->MakeSpinCtrlDouble(GetParentWindow(AD_szStarTracking), wxID_ANY, wxEmptyString, wxDefaultPosition,
        wxSize(width, -1), wxSP_ARROW_KEYS, minHFD, 10.0, pGuider->GetMinStarHFDFloor(), 0.5);
    m_MinHFD->SetDigits(1);
    wxSizer *pHFD = MakeLabeledControl(AD_szStarTracking, _("Minimum star HFD (pixels)"), m_MinHFD,
        _("The minimum star HFD (size) that will be used for identifying a guide star. "
        "This setting can be used to prevent PHD2 from guiding on a hot pixel. "
        "Use the Star Profile Tool to measure the HFD of a hot pixel and set the min HFD threshold "
        "a bit higher. When the HFD falls below this level, the hot pixel will be ignored."));

    m_MaxHFD = pFrame->MakeSpinCtrlDouble(GetParentWindow(AD_szStarTracking), wxID_ANY, wxEmptyString, wxDefaultPosition,
        wxSize(width, -1), wxSP_ARROW_KEYS, minHFD + 2.0, 10.0, 5.0, 0.5);
    m_MaxHFD->SetDigits(1);
    wxSizer *pMaxHFD = MakeLabeledControl(AD_szStarTracking, _("Maximum star HFD (pixels)"), m_MaxHFD,
        _("The maximum star HFD that will be used for identifying a guide star. "
        "This setting can be used to prevent PHD2 from choosing a large clump of sensor noise, adjacent faint stars, "
        "internal reflections, or comet heads as guide stars."));

    wxString ary[] = { _("Auto"), _T("1"), _T("2"), _T("3") };
    m_autoSelDownsample = new wxChoice(GetParentWindow(AD_szStarTracking), wxID_ANY, wxDefaultPosition, wxDefaultSize, WXSIZEOF(ary), ary);
    wxSizer *dsamp = MakeLabeledControl(AD_szStarTracking, _("Auto-selection frame downsample"), m_autoSelDownsample,
        _("Downsampling factor for star auto-selection camera frames. Choose a value greater than 1 if star "
        "auto-selection is failing to recognize misshapen guide stars."));

    m_pBeepForLostStarCtrl = new wxCheckBox(GetParentWindow(AD_cbBeepForLostStar), wxID_ANY, _("Beep on lost star"));
    m_pBeepForLostStarCtrl->SetToolTip(_("Issue an audible alarm any time the guide star is lost"));

    m_pUseMultiStars = new wxCheckBox(GetParentWindow(AD_szStarTracking), MULTI_STAR_ENABLE, _("Use multiple stars"));
    m_pUseMultiStars->SetToolTip(_("Use multiple guide stars if they are available"));
    GetParentWindow(AD_szStarTracking)->Bind(wxEVT_COMMAND_CHECKBOX_CLICKED, &GuiderMultiStarConfigDialogCtrlSet::OnMultiStarChecked, this, MULTI_STAR_ENABLE);
    width = StringWidth(_T("100.0"));

    m_MinSNR = pFrame->MakeSpinCtrlDouble(GetParentWindow(AD_szStarTracking), wxID_ANY, wxEmptyString, wxDefaultPosition,
        wxSize(width, -1), wxSP_ARROW_KEYS, 6.0, 200.0, 6.0, 2.0);
    m_MinSNR->SetDigits(0);
    wxSizer *pSNR = MakeLabeledControl(AD_szStarTracking, _("Minimum star SNR for AutoFind"), m_MinSNR,
        _("The minimum star SNR that will be used for auto-selecting guide stars. "
        "This setting can be used to discourage PHD2 from choosing a guide star you know will be too faint for sustained guiding. "
        "This setting applies to both the primary guide star and candidate secondary stars in multi-star guiding. "
        "If this constraint cannot be met, a saturated or near-saturated star may be selected."));

    wxFlexGridSizer *pTrackingParams = new wxFlexGridSizer(3, 2, 8, 15);
    pTrackingParams->Add(pSearchRegion, wxSizerFlags(0).Border(wxTOP, 12));
    pTrackingParams->Add(pStarMass, wxSizerFlags(0).Border(wxLEFT, 75));
    pTrackingParams->Add(pHFD, wxSizerFlags().Border(wxTOP, 3));
    pTrackingParams->Add(pSNR, wxSizerFlags().Border(wxLEFT, 75));
    pTrackingParams->Add(pMaxHFD, wxSizerFlags().Border(wxTOP, 4));
    pTrackingParams->Add(m_pUseMultiStars, wxSizerFlags(0).Border(wxLEFT, 75));
    pTrackingParams->Add(m_pBeepForLostStarCtrl, wxSizerFlags().Border(wxTOP, 3));
    pTrackingParams->Add(dsamp, wxSizerFlags().Border(wxTOP, 3).Right());

    AddGroup(CtrlMap, AD_szStarTracking, pTrackingParams);
}

GuiderMultiStarConfigDialogCtrlSet::~GuiderMultiStarConfigDialogCtrlSet()
{

}

void GuiderMultiStarConfigDialogCtrlSet::LoadValues()
{
    bool starMassEnabled = m_pGuiderMultiStar->GetMassChangeThresholdEnabled();
    m_pEnableStarMassChangeThresh->SetValue(starMassEnabled);
    m_pMassChangeThreshold->Enable(starMassEnabled);
    m_pMassChangeThreshold->SetValue(100.0 * m_pGuiderMultiStar->GetMassChangeThreshold());
    m_pSearchRegion->SetValue(m_pGuiderMultiStar->GetSearchRegion());
    m_MinHFD->SetValue(m_pGuiderMultiStar->GetMinStarHFD());
    m_MinSNR->SetValue(m_pGuiderMultiStar->GetAFMinStarSNR());
    m_MaxHFD->SetValue(m_pGuiderMultiStar->GetMaxStarHFD());
    m_autoSelDownsample->SetSelection(m_pGuiderMultiStar->GetAutoSelDownsample());
    m_pBeepForLostStarCtrl->SetValue(pFrame->GetBeepForLostStar());
    m_pUseMultiStars->SetValue(m_pGuiderMultiStar->GetMultiStarMode());
    GuiderConfigDialogCtrlSet::LoadValues();
}

void GuiderMultiStarConfigDialogCtrlSet::UnloadValues()
{
    m_pGuiderMultiStar->SetMassChangeThresholdEnabled(m_pEnableStarMassChangeThresh->GetValue());
    m_pGuiderMultiStar->SetMassChangeThreshold(m_pMassChangeThreshold->GetValue() / 100.0);
    m_pGuiderMultiStar->SetSearchRegion(m_pSearchRegion->GetValue());
    double min_hfd = m_MinHFD->GetValue();
    m_pGuiderMultiStar->SetMinStarHFD(min_hfd);
    m_pGuiderMultiStar->SetMaxStarHFD(wxMax(m_MaxHFD->GetValue(), min_hfd + 2.0));
    m_pGuiderMultiStar->SetAFMinStarSNR(m_MinSNR->GetValue());
    m_pGuiderMultiStar->SetAutoSelDownsample(m_autoSelDownsample->GetSelection());
    if (m_pBeepForLostStarCtrl->GetValue() != pFrame->GetBeepForLostStar())
        pFrame->SetBeepForLostStar(m_pBeepForLostStarCtrl->GetValue());
    m_pGuiderMultiStar->SetMultiStarMode(m_pUseMultiStars->GetValue());
    GuiderConfigDialogCtrlSet::UnloadValues();
}

void GuiderMultiStarConfigDialogCtrlSet::OnMultiStarChecked(wxCommandEvent& evt)
{

}
void GuiderMultiStarConfigDialogCtrlSet::OnStarMassEnableChecked(wxCommandEvent& event)
{
    m_pMassChangeThreshold->Enable(event.IsChecked());
}

using namespace cv;

void GuiderMultiStar::CalcLineParams(CircleDescriptor p1, CircleDescriptor p2)
{
    if ((p1.radius == 0) || (p2.radius == 0))
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

// An algorithm to find eclipse center
void GuiderMultiStar::FindEclipseCenter(CircleDescriptor& eclipseCenter, CircleDescriptor& smallestCircle, std::vector<Point2f>& eclipseContour, int minRadius, int maxRadius)
{
    int bestScore = 0;
    int searchRadius = smallestCircle.radius / 4;
    Point pointToMeasure;

    // When center of mass (centroid) wasn't found use smallest circle as best guess
    if (!m_DiameterLineParameters.valid)
    {
        eclipseCenter = smallestCircle;
        return;
    }

    if (!m_DiameterLineParameters.vertical && (fabs(m_DiameterLineParameters.slope) < 1.0))
    {
        // Search along x-axis when line slope is below 45 degrees
        for (pointToMeasure.x = smallestCircle.x - searchRadius; pointToMeasure.x <= smallestCircle.x + searchRadius; pointToMeasure.x++)
        {
            // Count number of points of the contour which are equidistant from pointToMeasure.
            // The point with maximum score is identified as eclipse center.
            pointToMeasure.y = m_DiameterLineParameters.slope * pointToMeasure.x + m_DiameterLineParameters.b;
            float maxDist = 0;
            float minDist = 999999;
            int scorePoints = 0;
            for (const Point& contourPoint : eclipseContour)
            {
                float distance = norm(contourPoint - pointToMeasure);
                if ((distance <= maxRadius) && (distance >= minRadius))
                {
                    if (maxDist < distance)
                        maxDist = distance;
                    if (minDist > distance)
                        minDist = distance;
                }
            }
            for (const Point& contourPoint : eclipseContour)
            {
                float distance = norm(contourPoint - pointToMeasure);
                if (fabs(distance - maxDist) < 5)
                    scorePoints++;
            }
            if (scorePoints > bestScore)
            {
                bestScore = scorePoints;
                eclipseCenter.radius = maxDist;
                eclipseCenter.x = pointToMeasure.x;
                eclipseCenter.y = pointToMeasure.y;
            }
        }
    }
    else
    {
        // Search along y-axis when slope is above 45 degrees
        for (pointToMeasure.y = smallestCircle.y - searchRadius; pointToMeasure.y <= smallestCircle.y + searchRadius; pointToMeasure.y++)
        {
            // Count number of points of the contour which are equidistant from pointToMeasure.
            // The point with maximum score is identified as eclipse center.
            if (m_DiameterLineParameters.vertical)
                pointToMeasure.x = smallestCircle.x;
            else
                pointToMeasure.x = (pointToMeasure.y - m_DiameterLineParameters.b) / m_DiameterLineParameters.slope;
            float maxDist = 0;
            float minDist = 999999;
            int scorePoints = 0;
            for (const Point& contourPoint : eclipseContour)
            {
                float distance = norm(contourPoint - pointToMeasure);
                if ((distance <= maxRadius) && (distance >= minRadius))
                {
                    if (maxDist < distance)
                        maxDist = distance;
                    if (minDist > distance)
                        minDist = distance;
                }
            }
            for (const Point& contourPoint : eclipseContour)
            {
                float distance = norm(contourPoint - pointToMeasure);
                if (fabs(distance - maxDist) < 5)
                    scorePoints++;
            }
            if (scorePoints > bestScore)
            {
                bestScore = scorePoints;
                eclipseCenter.radius = maxDist;
                eclipseCenter.x = pointToMeasure.x;
                eclipseCenter.y = pointToMeasure.y;
            }
        }
    }
}

// Find a minimum enclosing circle of the contour and also its center of mass
void GuiderMultiStar::FindCenters(CvSeq* contours, CircleDescriptor& bestCentroid, CircleDescriptor& smallestCircle, std::vector<Point2f>& eclipseContour, int minRadius, int maxRadius)
{
    CvSeq* smallestContour = NULL;
    smallestCircle.radius = 999999;

    for (CvSeq* contour = contours; contour != NULL; contour = contour->h_next)
    {
        CvPoint2D32f circleCenter;
        float circle_radius;
        if (cvMinEnclosingCircle(contour, &circleCenter, &circle_radius))
            if ((cvRound(circle_radius) <= maxRadius) &&
                (cvRound(circle_radius) >= minRadius))
            {
                if (circle_radius < smallestCircle.radius)
                {
                    smallestContour = contour;
                    smallestCircle.x = circleCenter.x;
                    smallestCircle.y = circleCenter.y;
                    smallestCircle.radius = circle_radius;
                }
            }
    }

    eclipseContour.clear();
    if (smallestContour)
    {
        // Calculate moment
        Point2f centroid = { 0 };
        for (int i = 0; i < smallestContour->total; i++)
        {
            CvPoint* pt = CV_GET_SEQ_ELEM(CvPoint, smallestContour, i);
            eclipseContour.push_back(cv::Point(pt->x, pt->y));
        }
        Moments mu = moments(eclipseContour, false);
        if (mu.m00 > 0)
        {
            bestCentroid.x = mu.m10 / mu.m00;
            bestCentroid.y = mu.m01 / mu.m00;
            bestCentroid.radius = smallestCircle.radius;
        }
        else
            bestCentroid.radius = 0;
    }
    else
        smallestCircle.radius = 0;
}

bool GuiderMultiStar::FindPlanet(const usImage *pImage, bool autoSelect)
{
    Point2f clickedPoint = { (float) m_Planet.clicked_x, (float) m_Planet.clicked_y };
    if (autoSelect)
        m_Planet.clicked = false;

    // Do not run out of real time
    if (m_PlanetWatchdog.Time() < 100)
    {
        Debug.Write("Warning: FindPlanet: running out of real time\n");
        return m_Planet.detected;
    }

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
        return false;
    }

    // Use ROI as optimization
    bool usingRoi = false;
    int offsetX = 0;
    int offsetY = 0;
    Mat FullFrame(pImage->Size.GetHeight(), pImage->Size.GetWidth(), format, pImage->ImageData);
    Mat RoiFrame;
    Mat img8;
    if (!autoSelect && GetRoiEnableState() &&
        (m_Planet.roi_radius > 0) &&
        (m_Planet.frame_width == pImage->Size.GetWidth()) &&
        (m_Planet.frame_height == pImage->Size.GetHeight()))
    {
        offsetX = m_Planet.center_x - m_Planet.roi_radius;
        if (offsetX < 0)
            offsetX = 0;
        offsetY = m_Planet.center_y - m_Planet.roi_radius;
        if (offsetY < 0)
            offsetY = 0;
        int w = m_Planet.roi_radius * 2;
        if (w > pImage->Size.GetWidth() - offsetX)
            w = pImage->Size.GetWidth() - offsetX;
        int h = m_Planet.roi_radius * 2;
        if (h > pImage->Size.GetHeight() - offsetY)
            h = pImage->Size.GetHeight() - offsetY;
        Rect roiRect(offsetX, offsetY, w, h);
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

    bool eclipse_mode = GetEclipseMode();
    if (!eclipse_mode)
    {
        double minDist = GetPlanetaryParam_minDist();
        double param1 = GetPlanetaryParam_param1();
        double param2 = GetPlanetaryParam_param2();

        // Find circles matching given criteria
        Debug.Write(wxString::Format("Start detection of planetary disk (roi:%d mind=%.1f,p1=%.1f,p2=%.1f,minr=%d,maxr=%d)\n", usingRoi, minDist, param1, param2, minRadius, maxRadius));
        vector<Vec3f> circles;
        HoughCircles(img8, circles, CV_HOUGH_GRADIENT, 1.0, minDist, param1, param2, minRadius, maxRadius);

        // Find and use largest circle from the detected set of circles
        Vec3f center = { 0, 0, 0 };
        for (const auto& c: circles)
        {
            Point2f circlePoint = { offsetX + c[0], offsetY + c[1] };
            float distanceAllowed = c[2] * 1.5;
            if ((c[2] > center[2]) && (!m_Planet.clicked || autoSelect || norm(clickedPoint - circlePoint) <= distanceAllowed))
                center = c;
        }

        Debug.Write(wxString::Format("End detection of planetary disk: %d circles detected, r=%d x=%.1f y=%.1f\n", circles.size(), cvRound(center[2]), center[0], center[1]));
        m_PlanetWatchdog.Start();

        if (center[2])
        {
            m_Planet.frame_width = pImage->Size.GetWidth();
            m_Planet.frame_height = pImage->Size.GetHeight();
            m_Planet.center_x = offsetX + center[0];
            m_Planet.center_y = offsetY + center[1];
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

        // Create wxImage from the OpenCV Mat to be presented as 
        // a visual aid for tunning of the edge threshold parameters.
        if (GetPlanetaryThresholdVisual())
        {
            m_Planet.sync_lock.Lock();
            if (m_Planet.eclipse_edges)
            {
                m_Planet.eclipse_edges_valid = false;
                delete m_Planet.eclipse_edges;
            }
            size_t dataSize = dilatedEdges.rows * dilatedEdges.cols * dilatedEdges.channels();
            m_Planet.eclipse_edges = new unsigned char[dataSize];
            m_Planet.rows = dilatedEdges.rows;
            m_Planet.cols = dilatedEdges.cols;
            m_Planet.offset_x = offsetX;
            m_Planet.offset_y = offsetY;
            memcpy(m_Planet.eclipse_edges, dilatedEdges.data, dataSize);
            m_Planet.eclipse_edges_valid = true;
            m_Planet.sync_lock.Unlock();
        }

        // Find contours
        IplImage thresholded = IplImage(dilatedEdges);
        CvMemStorage *storage = cvCreateMemStorage(0);
        CvSeq *contours = NULL;
        cvFindContours(&thresholded, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

        // Find the smallest circle encompassing contour of the eclipsed disk
        // and also center of mass within the contour.
        std::vector<Point2f> eclipseContour;
        CircleDescriptor smallestCircle = { 0 };
        CircleDescriptor Centroid = { 0 };
        CircleDescriptor eclipseCenter = { 0 };
        FindCenters(contours, Centroid, smallestCircle, eclipseContour, minRadius, maxRadius);

        // Look for a point along the line connecting centers of the smallest circle and center of mass
        // which is most equidistant from the outmost edge of the contour. Consider this point as 
        // the best match for eclipse central point.
        CalcLineParams(smallestCircle, Centroid);
        FindEclipseCenter(eclipseCenter, smallestCircle, eclipseContour, minRadius, maxRadius);

        // Free allocated storage
        cvReleaseMemStorage(&storage);
        Debug.Write(wxString::Format("End detection of eclipsed disk: t=%d r=%.1f, x=%.1f, y=%.1f\n", m_PlanetWatchdog.Time(), eclipseCenter.radius, offsetX + eclipseCenter.x, offsetY + eclipseCenter.y));

        m_PlanetWatchdog.Start();
        // When user clicks a point in the main window, discard detected features 
        // that are far away from it, similar to manual selection of stars in PHD2.
        Point2f circlePoint = { offsetX + eclipseCenter.x, offsetY + eclipseCenter.y };
        float distanceAllowed = eclipseCenter.radius * 1.5;
        if (!autoSelect && m_Planet.clicked && (eclipseCenter.radius > 0) && norm(clickedPoint - circlePoint) > distanceAllowed)
        {
            eclipseCenter.radius = 0;
            Debug.Write(_("Warning: detected object discarded: too far from user's selection\n"));
        }

        if (eclipseCenter.radius > 0)
        {
            m_Planet.frame_width = pImage->Size.GetWidth();
            m_Planet.frame_height = pImage->Size.GetHeight();
            m_Planet.center_x = offsetX + eclipseCenter.x;
            m_Planet.center_y = offsetY + eclipseCenter.y;
            m_Planet.radius = eclipseCenter.radius;
            m_Planet.roi_radius = (m_Planet.radius * 3) / 2;
            m_Planet.detected = true;
            return true;
        }
    }

    m_Planet.detected = false;
    m_Planet.roi_radius = 0;
    return false;
}