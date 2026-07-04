/*
 *  event_server.cpp
 *  PHD Guiding
 *
 *  Created by Andy Galasso.
 *  Copyright (c) 2013 Andy Galasso.
 *  Solar branch extensions Copyright (c) 2024-2026 Leo Shatz.
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
#include "guiding_assistant.h"
#include "frame_export.h"

#include <cmath>
#include <wx/sstream.h>
#include <wx/sckstrm.h>
#include <sstream>
#include <string.h>

EventServer EvtServer;

// Tracks nested event-server RPC dispatch on the UI thread.
static thread_local int s_rpcDepth = 0;

namespace {
struct RpcEntryGuard
{
    RpcEntryGuard() { ++s_rpcDepth; }
    ~RpcEntryGuard() { --s_rpcDepth; }
};
} // namespace

bool EventServer::InRpcCall()
{
    return s_rpcDepth > 0;
}

// clang-format off
wxBEGIN_EVENT_TABLE(EventServer, wxEvtHandler)
    EVT_SOCKET(EVENT_SERVER_ID, EventServer::OnEventServerEvent)
    EVT_SOCKET(EVENT_SERVER_CLIENT_ID, EventServer::OnEventServerClientEvent)
wxEND_EVENT_TABLE();
// clang-format on

enum
{
    MSG_PROTOCOL_VERSION = 1,
};

#define MSG_EXTENTED_PROTOCOL_VERSION_SOLAR1 "solar.1"
#define MSG_EXTENTED_PROTOCOL_VERSION_SOLAR2 "solar.2"
#define MSG_EXTENTED_PROTOCOL_VERSION_SOLAR3 "solar.3"

enum EXT_PROTOCOL_VERSION
{
    EXT_PROTOCOL_UNSPECIFIED = 0,
    EXT_PROTOCOL_SOLAR1 = 1,
    EXT_PROTOCOL_SOLAR2 = 2,
    EXT_PROTOCOL_SOLAR3 = 3,
};

static const char *ext_protocol_version_name(EXT_PROTOCOL_VERSION version)
{
    switch (version)
    {
    case EXT_PROTOCOL_SOLAR3:
        return MSG_EXTENTED_PROTOCOL_VERSION_SOLAR3;
    case EXT_PROTOCOL_SOLAR2:
        return MSG_EXTENTED_PROTOCOL_VERSION_SOLAR2;
    default:
        return MSG_EXTENTED_PROTOCOL_VERSION_SOLAR1;
    }
}

static bool protocol_is_solar2(EXT_PROTOCOL_VERSION protocol)
{
    return protocol >= EXT_PROTOCOL_SOLAR2;
}

static const wxString literal_null("null");
static const wxString literal_true("true");
static const wxString literal_false("false");

static wxString state_name(EXPOSED_STATE st)
{
    switch (st)
    {
    case EXPOSED_STATE_NONE:
        return "Stopped";
    case EXPOSED_STATE_SELECTED:
        return "Selected";
    case EXPOSED_STATE_CALIBRATING:
        return "Calibrating";
    case EXPOSED_STATE_GUIDING_LOCKED:
        return "Guiding";
    case EXPOSED_STATE_GUIDING_LOST:
        return "LostLock";
    case EXPOSED_STATE_PAUSED:
        return "Paused";
    case EXPOSED_STATE_LOOPING:
        return "Looping";
    default:
        return "Unknown";
    }
}

static wxString json_escape(const wxString& s)
{
    wxString t(s);
    static const wxString BACKSLASH("\\");
    static const wxString BACKSLASHBACKSLASH("\\\\");
    static const wxString DQUOT("\"");
    static const wxString BACKSLASHDQUOT("\\\"");
    static const wxString CR("\r");
    static const wxString BACKSLASHCR("\\r");
    static const wxString LF("\n");
    static const wxString BACKSLASHLF("\\n");
    t.Replace(BACKSLASH, BACKSLASHBACKSLASH);
    t.Replace(DQUOT, BACKSLASHDQUOT);
    t.Replace(CR, BACKSLASHCR);
    t.Replace(LF, BACKSLASHLF);
    return t;
}

template<char LDELIM, char RDELIM>
struct JSeq
{
    wxString m_s;
    bool m_first;
    bool m_closed;
    JSeq() : m_first(true), m_closed(false) { m_s << LDELIM; }
    void close()
    {
        m_s << RDELIM;
        m_closed = true;
    }
    wxString str()
    {
        if (!m_closed)
            close();
        return m_s;
    }
};

typedef JSeq<'[', ']'> JAry;
typedef JSeq<'{', '}'> JObj;

static JAry& operator<<(JAry& a, const wxString& str)
{
    if (a.m_first)
        a.m_first = false;
    else
        a.m_s << ',';
    a.m_s << str;
    return a;
}

static JAry& operator<<(JAry& a, double d)
{
    return a << wxString::Format("%.2f", d);
}

static JAry& operator<<(JAry& a, int i)
{
    return a << wxString::Format("%d", i);
}

static wxString json_format(const json_value *j)
{
    if (!j)
        return literal_null;

    switch (j->type)
    {
    default:
    case JSON_NULL:
        return literal_null;
    case JSON_OBJECT:
    {
        wxString ret("{");
        bool first = true;
        json_for_each(jj, j)
        {
            if (first)
                first = false;
            else
                ret << ",";
            ret << '"' << jj->name << "\":" << json_format(jj);
        }
        ret << "}";
        return ret;
    }
    case JSON_ARRAY:
    {
        wxString ret("[");
        bool first = true;
        json_for_each(jj, j)
        {
            if (first)
                first = false;
            else
                ret << ",";
            ret << json_format(jj);
        }
        ret << "]";
        return ret;
    }
    case JSON_STRING:
        return '"' + json_escape(j->string_value) + '"';
    case JSON_INT:
        return wxString::Format("%d", j->int_value);
    case JSON_FLOAT:
        return wxString::Format("%g", (double) j->float_value);
    case JSON_BOOL:
        return j->int_value ? literal_true : literal_false;
    }
}

struct NULL_TYPE
{
} NULL_VALUE;

// name-value pair
struct NV
{
    wxString n;
    wxString v;
    NV(const wxString& n_, const wxString& v_) : n(n_), v('"' + json_escape(v_) + '"') { }
    NV(const wxString& n_, const char *v_) : n(n_), v('"' + json_escape(v_) + '"') { }
    NV(const wxString& n_, const wchar_t *v_) : n(n_), v('"' + json_escape(v_) + '"') { }
    NV(const wxString& n_, int v_) : n(n_), v(wxString::Format("%d", v_)) { }
    NV(const wxString& n_, unsigned int v_) : n(n_), v(wxString::Format("%u", v_)) { }
    NV(const wxString& n_, double v_) : n(n_), v(wxString::Format("%g", v_)) { }
    NV(const wxString& n_, double v_, int prec) : n(n_), v(wxString::Format("%.*f", prec, v_)) { }
    NV(const wxString& n_, bool v_) : n(n_), v(v_ ? literal_true : literal_false) { }
    template<typename T>
    NV(const wxString& n_, const std::vector<T>& vec);
    NV(const wxString& n_, JAry& ary) : n(n_), v(ary.str()) { }
    NV(const wxString& n_, JObj& obj) : n(n_), v(obj.str()) { }
    NV(const wxString& n_, const json_value *v_) : n(n_), v(json_format(v_)) { }
    NV(const wxString& n_, const PHD_Point& p) : n(n_)
    {
        JAry ary;
        ary << p.X << p.Y;
        v = ary.str();
    }
    NV(const wxString& n_, const wxPoint& p) : n(n_)
    {
        JAry ary;
        ary << p.x << p.y;
        v = ary.str();
    }
    NV(const wxString& n_, const wxSize& s) : n(n_)
    {
        JAry ary;
        ary << s.x << s.y;
        v = ary.str();
    }
    NV(const wxString& n_, const wxRect& r) : n(n_)
    {
        JAry ary;
        ary << r.x << r.y << r.width << r.height;
        v = ary.str();
    }
    NV(const wxString& n_, const NULL_TYPE& nul) : n(n_), v(literal_null) { }
};

template<typename T>
NV::NV(const wxString& n_, const std::vector<T>& vec) : n(n_)
{
    std::ostringstream os;
    os << '[';
    for (unsigned int i = 0; i < vec.size(); i++)
    {
        if (i != 0)
            os << ',';
        os << vec[i];
    }
    os << ']';
    v = os.str();
}

static JObj& operator<<(JObj& j, const NV& nv)
{
    if (j.m_first)
        j.m_first = false;
    else
        j.m_s << ',';
    j.m_s << '"' << nv.n << "\":" << nv.v;
    return j;
}

static NV NVMount(const Mount *mount)
{
    return NV("Mount", mount->Name());
}

static JObj& operator<<(JObj& j, const PHD_Point& pt)
{
    return j << NV("X", pt.X, 3) << NV("Y", pt.Y, 3);
}

static JAry& operator<<(JAry& a, JObj& j)
{
    return a << j.str();
}

struct Ev : public JObj
{
    Ev(const wxString& event)
    {
        double const now = ::wxGetUTCTimeMillis().ToDouble() / 1000.0;
        *this << NV("Event", event) << NV("Timestamp", now, 3) << NV("Host", wxGetHostName())
              << NV("Inst", wxGetApp().GetInstanceNumber());
    }
};

static Ev ev_message_version(EXT_PROTOCOL_VERSION protocol)
{
    Ev ev("Version");
    ev << NV("PHDVersion", PHDVERSION) << NV("PHDSubver", PHDSUBVER) << NV("OverlapSupport", true)
       << NV("MsgVersion", MSG_PROTOCOL_VERSION) << NV("MsgExtVersion", ext_protocol_version_name(protocol));
    return ev;
}

static Ev ev_set_lock_position(const PHD_Point& xy)
{
    Ev ev("LockPositionSet");
    ev << xy;
    return ev;
}

static Ev ev_calibration_complete(const Mount *mount, CalibrationIssueType issue)
{
    Ev ev("CalibrationComplete");
    ev << NVMount(mount);

    if (mount->IsStepGuider())
    {
        ev << NV("Limit", mount->GetAoMaxPos());
    }

    wxString issueStr;
    switch (issue)
    {
    case CI_Steps:
        issueStr = "Steps";
        break;
    case CI_Angle:
        issueStr = "Angle";
        break;
    case CI_Rates:
        issueStr = "Rates";
        break;
    case CI_Different:
        issueStr = "Different";
        break;
    default:
        issueStr = "None";
        break;
    }
    ev << NV("Issue", issueStr);

    return ev;
}

static Ev ev_star_selected(const PHD_Point& pos)
{
    Ev ev("StarSelected");
    ev << pos;
    return ev;
}

static Ev ev_start_guiding()
{
    return Ev("StartGuiding");
}

static Ev ev_paused()
{
    return Ev("Paused");
}

static Ev ev_start_calibration(const Mount *mount)
{
    Ev ev("StartCalibration");
    ev << NVMount(mount);
    return ev;
}

static Ev ev_app_state(EXPOSED_STATE st = Guider::GetExposedState())
{
    Ev ev("AppState");
    ev << NV("State", state_name(st));
    return ev;
}

static Ev ev_settling(double distance, double time, double settleTime, bool starLocked)
{
    Ev ev("Settling");

    ev << NV("Distance", distance, 2) << NV("Time", time, 1) << NV("SettleTime", settleTime, 1) << NV("StarLocked", starLocked);

    return ev;
}

static Ev ev_settle_done(const wxString& errorMsg, int settleFrames, int droppedFrames)
{
    Ev ev("SettleDone");

    int status = errorMsg.IsEmpty() ? 0 : 1;

    ev << NV("Status", status);

    if (status != 0)
    {
        ev << NV("Error", errorMsg);
    }

    ev << NV("TotalFrames", settleFrames) << NV("DroppedFrames", droppedFrames);

    return ev;
}

struct ClientReadBuf
{
    enum
    {
        SIZE = 1024
    };
    char m_buf[SIZE];
    char *dest;

    ClientReadBuf() { reset(); }
    char *buf() { return &m_buf[0]; }
    size_t len() const { return dest - &m_buf[0]; }
    size_t avail() const { return &m_buf[SIZE] - dest; }
    void reset() { dest = &m_buf[0]; }
};

struct ClientData
{
    wxSocketClient *cli;
    int refcnt;
    EXT_PROTOCOL_VERSION extProtocol;
    ClientReadBuf rdbuf;
    wxMutex wrlock;

    ClientData(wxSocketClient *cli_) : cli(cli_), refcnt(1), extProtocol(EXT_PROTOCOL_SOLAR1) { }
    void AddRef() { ++refcnt; }
    void RemoveRef()
    {
        if (--refcnt == 0)
        {
            cli->Destroy();
            delete this;
        }
    }
};

struct ClientDataGuard
{
    ClientData *cd;
    ClientDataGuard(wxSocketClient *cli) : cd((ClientData *) cli->GetClientData()) { cd->AddRef(); }
    ~ClientDataGuard() { cd->RemoveRef(); }
    ClientData *operator->() const { return cd; }
};

inline static wxMutex *client_wrlock(wxSocketClient *cli)
{
    return &((ClientData *) cli->GetClientData())->wrlock;
}

static ClientData *client_data(wxSocketClient *cli)
{
    return cli ? static_cast<ClientData *>(cli->GetClientData()) : nullptr;
}

static EXT_PROTOCOL_VERSION client_ext_protocol(wxSocketClient *cli)
{
    ClientData *cd = client_data(cli);
    return cd ? cd->extProtocol : EXT_PROTOCOL_SOLAR1;
}

static wxString SockErrStr(wxSocketError e)
{
    switch (e)
    {
    case wxSOCKET_NOERROR:
        return "";
    case wxSOCKET_INVOP:
        return "Invalid operation";
    case wxSOCKET_IOERR:
        return "Input / Output error";
    case wxSOCKET_INVADDR:
        return "Invalid address";
    case wxSOCKET_INVSOCK:
        return "Invalid socket(uninitialized)";
    case wxSOCKET_NOHOST:
        return "No corresponding host";
    case wxSOCKET_INVPORT:
        return "Invalid port";
    case wxSOCKET_WOULDBLOCK:
        return "operation would block";
    case wxSOCKET_TIMEDOUT:
        return "timeout expired";
    case wxSOCKET_MEMERR:
        return "Memory exhausted";
    default:
        return wxString::Format("unknown socket error %d", e);
    }
}

static void send_buf(wxSocketClient *client, const wxCharBuffer& buf)
{
    wxMutexLocker lock(*client_wrlock(client));
    client->Write(buf.data(), buf.length());
    if (client->LastWriteCount() != buf.length())
    {
        Debug.Write(wxString::Format("evsrv: cli %p short write %u/%u %s\n", client, client->LastWriteCount(),
                                     (unsigned int) buf.length(),
                                     SockErrStr(client->Error() ? client->LastError() : wxSOCKET_NOERROR)));
    }
}

static void do_notify1(wxSocketClient *client, const JAry& ary)
{
    send_buf(client, (JAry(ary).str() + "\r\n").ToUTF8());
}

static void do_notify1(wxSocketClient *client, const JObj& j)
{
    send_buf(client, (JObj(j).str() + "\r\n").ToUTF8());
}

static void do_notify(const EventServer::CliSockSet& cli, const JObj& jj)
{
    wxCharBuffer buf = (JObj(jj).str() + "\r\n").ToUTF8();

    for (EventServer::CliSockSet::const_iterator it = cli.begin(); it != cli.end(); ++it)
    {
        send_buf(*it, buf);
    }
}

inline static void simple_notify(wxMutex& lock, const EventServer::CliSockSet& cli, const wxString& ev)
{
    wxMutexLocker lck(lock);
    if (!cli.empty())
        do_notify(cli, Ev(ev));
}

inline static void simple_notify_ev(wxMutex& lock, const EventServer::CliSockSet& cli, const Ev& ev)
{
    wxMutexLocker lck(lock);
    if (!cli.empty())
        do_notify(cli, ev);
}

#define SIMPLE_NOTIFY(s) simple_notify(m_clientsLock, m_eventServerClients, s)
#define SIMPLE_NOTIFY_EV(ev) simple_notify_ev(m_clientsLock, m_eventServerClients, ev)

static void send_catchup_events(wxSocketClient *cli)
{
    EXPOSED_STATE st = Guider::GetExposedState();

    do_notify1(cli, ev_message_version(EXT_PROTOCOL_SOLAR1));

    if (pFrame->pGuider)
    {
        if (pFrame->pGuider->LockPosition().IsValid())
            do_notify1(cli, ev_set_lock_position(pFrame->pGuider->LockPosition()));

        if (pFrame->pGuider->CurrentPosition().IsValid())
            do_notify1(cli, ev_star_selected(pFrame->pGuider->CurrentPosition()));
    }

    if (pMount && pMount->IsCalibrated())
        do_notify1(cli, ev_calibration_complete(pMount, pMount->GetCalIssue()));

    if (pSecondaryMount && pSecondaryMount->IsCalibrated())
        do_notify1(cli, ev_calibration_complete(pSecondaryMount, pSecondaryMount->GetCalIssue()));

    if (st == EXPOSED_STATE_GUIDING_LOCKED)
    {
        do_notify1(cli, ev_start_guiding());
    }
    else if (st == EXPOSED_STATE_CALIBRATING)
    {
        Mount *mount = pMount;
        if (pFrame->pGuider->GetState() == STATE_CALIBRATING_SECONDARY)
            mount = pSecondaryMount;
        do_notify1(cli, ev_start_calibration(mount));
    }
    else if (st == EXPOSED_STATE_PAUSED)
    {
        do_notify1(cli, ev_paused());
    }

    do_notify1(cli, ev_app_state());
}

static void destroy_client(wxSocketClient *cli)
{
    ClientData *buf = (ClientData *) cli->GetClientData();
    buf->RemoveRef();
}

static void drain_input(wxSocketInputStream& sis)
{
    while (sis.CanRead())
    {
        char buf[1024];
        if (sis.Read(buf, sizeof(buf)).LastRead() == 0)
            break;
    }
}

enum
{
    JSONRPC_PARSE_ERROR = -32700,
    JSONRPC_INVALID_REQUEST = -32600,
    JSONRPC_METHOD_NOT_FOUND = -32601,
    JSONRPC_INVALID_PARAMS = -32602,
    JSONRPC_INTERNAL_ERROR = -32603,
};

static NV jrpc_error(int code, const wxString& msg)
{
    JObj err;
    err << NV("code", code) << NV("message", msg);
    return NV("error", err);
}

template<typename T>
static NV jrpc_result(const T& t)
{
    return NV("result", t);
}

template<typename T>
static NV jrpc_result(T& t)
{
    return NV("result", t);
}

static NV jrpc_id(const json_value *id)
{
    return NV("id", id);
}

struct JRpcResponse : public JObj
{
    JRpcResponse() { *this << NV("jsonrpc", "2.0"); }
};

static wxString parser_error(const JsonParser& parser)
{
    return wxString::Format("invalid JSON request: %s on line %d at \"%.12s...\"", parser.ErrorDesc(), parser.ErrorLine(),
                            parser.ErrorPos());
}

static void parse_request(const json_value *req, const json_value **pmethod, const json_value **pparams, const json_value **pid)
{
    *pmethod = *pparams = *pid = 0;

    if (req)
    {
        json_for_each(t, req)
        {
            if (t->name)
            {
                if (t->type == JSON_STRING && strcmp(t->name, "method") == 0)
                    *pmethod = t;
                else if (strcmp(t->name, "params") == 0)
                    *pparams = t;
                else if (strcmp(t->name, "id") == 0)
                    *pid = t;
            }
        }
    }
}

// paranoia
#define VERIFY_GUIDER(response)                                                                                                \
    do                                                                                                                         \
    {                                                                                                                          \
        if (!pFrame || !pFrame->pGuider)                                                                                       \
        {                                                                                                                      \
            response << jrpc_error(1, "internal error");                                                                       \
            return;                                                                                                            \
        }                                                                                                                      \
    } while (0)

static void deselect_star(JObj& response, const json_value *params)
{
    VERIFY_GUIDER(response);
    pFrame->pGuider->Reset(true);
    response << jrpc_result(0);
}

static void get_exposure(JObj& response, const json_value *params)
{
    response << jrpc_result(pFrame->RequestedExposureDuration());
}

static void get_exposure_durations(JObj& response, const json_value *params)
{
    const std::vector<int>& exposure_durations = pFrame->GetExposureDurations();
    response << jrpc_result(exposure_durations);
}

static void get_profiles(JObj& response, const json_value *params)
{
    JAry ary;
    wxArrayString names = pConfig->ProfileNames();
    for (unsigned int i = 0; i < names.size(); i++)
    {
        wxString name = names[i];
        int id = pConfig->GetProfileId(name);
        if (id)
        {
            JObj t;
            t << NV("id", id) << NV("name", name);
            if (id == pConfig->GetCurrentProfileId())
                t << NV("selected", true);
            ary << t;
        }
    }
    response << jrpc_result(ary);
}

struct Params
{
    std::map<std::string, const json_value *> dict;

    void Init(const char *names[], size_t nr_names, const json_value *params)
    {
        if (!params)
            return;
        if (params->type == JSON_ARRAY)
        {
            const json_value *jv = params->first_child;
            for (size_t i = 0; jv && i < nr_names; i++, jv = jv->next_sibling)
            {
                const char *name = names[i];
                dict.insert(std::make_pair(std::string(name), jv));
            }
        }
        else if (params->type == JSON_OBJECT)
        {
            json_for_each(jv, params)
            {
                dict.insert(std::make_pair(std::string(jv->name), jv));
            }
        }
    }
    Params(const char *n1, const json_value *params)
    {
        const char *n[] = { n1 };
        Init(n, 1, params);
    }
    Params(const char *n1, const char *n2, const json_value *params)
    {
        const char *n[] = { n1, n2 };
        Init(n, 2, params);
    }
    Params(const char *n1, const char *n2, const char *n3, const json_value *params)
    {
        const char *n[] = { n1, n2, n3 };
        Init(n, 3, params);
    }
    Params(const char *n1, const char *n2, const char *n3, const char *n4, const json_value *params)
    {
        const char *n[] = { n1, n2, n3, n4 };
        Init(n, 4, params);
    }
    Params(const char *n1, const char *n2, const char *n3, const char *n4, const char *n5, const json_value *params)
    {
        const char *n[] = { n1, n2, n3, n4, n5 };
        Init(n, 5, params);
    }
    Params(const char *n1, const char *n2, const char *n3, const char *n4, const char *n5, const char *n6,
           const json_value *params)
    {
        const char *n[] = { n1, n2, n3, n4, n5, n6 };
        Init(n, 6, params);
    }
    const json_value *param(const std::string& name) const
    {
        auto it = dict.find(name);
        return it == dict.end() ? 0 : it->second;
    }
};

static void set_extended_protocol(JObj& response, const json_value *params, ClientData *cd)
{
    Params p("version", params);
    const json_value *version = p.param("version");
    if (!version || version->type != JSON_STRING)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected version string param");
        return;
    }

    EXT_PROTOCOL_VERSION protocol;
    if (strcmp(version->string_value, MSG_EXTENTED_PROTOCOL_VERSION_SOLAR1) == 0)
        protocol = EXT_PROTOCOL_SOLAR1;
    else if (strcmp(version->string_value, MSG_EXTENTED_PROTOCOL_VERSION_SOLAR2) == 0)
        protocol = EXT_PROTOCOL_SOLAR2;
    else if (strcmp(version->string_value, MSG_EXTENTED_PROTOCOL_VERSION_SOLAR3) == 0)
        protocol = EXT_PROTOCOL_SOLAR3;
    else
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "unsupported protocol version");
        return;
    }

    if (!cd)
    {
        response << jrpc_error(JSONRPC_INTERNAL_ERROR, "client state not available");
        return;
    }

    cd->extProtocol = protocol;

    JObj rslt;
    rslt << NV("protocol", ext_protocol_version_name(protocol));
    if (protocol >= EXT_PROTOCOL_SOLAR3)
        rslt << NV("frame_export", FrameExport::Available());
    response << jrpc_result(rslt);
}

static void set_exposure(JObj& response, const json_value *params)
{
    Params p("exposure", params);
    const json_value *exp = p.param("exposure");

    if (!exp || exp->type != JSON_INT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected exposure param");
        return;
    }

    bool ok = pFrame->SetExposureDuration(exp->int_value, true);
    if (ok)
    {
        response << jrpc_result(0);
    }
    else
    {
        response << jrpc_error(1, "could not set exposure duration");
    }
}

static void get_profile(JObj& response, const json_value *params)
{
    int id = pConfig->GetCurrentProfileId();
    wxString name = pConfig->GetCurrentProfile();
    JObj t;
    t << NV("id", id) << NV("name", name);
    response << jrpc_result(t);
}

inline static void devstat(JObj& t, const char *dev, const wxString& name, bool connected)
{
    JObj o;
    t << NV(dev, o << NV("name", name) << NV("connected", connected));
}

static void get_current_equipment(JObj& response, const json_value *params)
{
    JObj t;

    if (pCamera)
        devstat(t, "camera", pCamera->Name, pCamera->Connected);

    Mount *mount = TheScope();
    if (mount)
        devstat(t, "mount", mount->Name(), mount->IsConnected());

    Mount *auxMount = pFrame->pGearDialog->AuxScope();
    if (auxMount)
        devstat(t, "aux_mount", auxMount->Name(), auxMount->IsConnected());

    Mount *ao = TheAO();
    if (ao)
        devstat(t, "AO", ao->Name(), ao->IsConnected());

    Rotator *rotator = pRotator;
    if (rotator)
        devstat(t, "rotator", rotator->Name(), rotator->IsConnected());

    response << jrpc_result(t);
}

static bool all_equipment_connected()
{
    return pCamera && pCamera->Connected && (!pMount || pMount->IsConnected()) &&
        (!pSecondaryMount || pSecondaryMount->IsConnected());
}

static void set_profile(JObj& response, const json_value *params)
{
    Params p("id", params);
    const json_value *id = p.param("id");
    if (!id || id->type != JSON_INT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected profile id param");
        return;
    }

    VERIFY_GUIDER(response);

    wxString errMsg;
    bool error = pFrame->pGearDialog->SetProfile(id->int_value, &errMsg);

    if (error)
    {
        response << jrpc_error(1, errMsg);
    }
    else
    {
        response << jrpc_result(0);
    }
}

static void get_connected(JObj& response, const json_value *params)
{
    response << jrpc_result(all_equipment_connected());
}

static void set_connected(JObj& response, const json_value *params)
{
    Params p("connected", params);
    const json_value *val = p.param("connected");
    if (!val || val->type != JSON_BOOL)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected connected boolean param");
        return;
    }

    VERIFY_GUIDER(response);

    wxString errMsg;
    bool error = val->int_value ? pFrame->pGearDialog->ConnectAll(&errMsg) : pFrame->pGearDialog->DisconnectAll(&errMsg);

    if (error)
    {
        response << jrpc_error(1, errMsg);
    }
    else
    {
        response << jrpc_result(0);
    }
}

static void get_calibrated(JObj& response, const json_value *params)
{
    bool calibrated = pMount && pMount->IsCalibrated() && (!pSecondaryMount || pSecondaryMount->IsCalibrated());
    response << jrpc_result(calibrated);
}

static bool float_param(const json_value *v, double *p)
{
    if (v->type == JSON_INT)
    {
        *p = (double) v->int_value;
        return true;
    }
    else if (v->type == JSON_FLOAT)
    {
        *p = v->float_value;
        return true;
    }

    return false;
}

static bool float_param(const char *name, const json_value *v, double *p)
{
    if (strcmp(name, v->name) != 0)
        return false;

    return float_param(v, p);
}

inline static bool bool_value(const json_value *v)
{
    return v->int_value ? true : false;
}

static bool bool_param(const json_value *jv, bool *val)
{
    if (jv->type != JSON_BOOL && jv->type != JSON_INT)
        return false;
    *val = bool_value(jv);
    return true;
}

static bool valid_sidereal_time(double siderealTime)
{
    return std::isfinite(siderealTime) && siderealTime >= 0.0 && siderealTime < 24.0;
}

static bool valid_ra(double ra)
{
    return std::isfinite(ra) && ra >= 0.0 && ra < 24.0;
}

static double hour_angle(double ra, double siderealTime)
{
    return norm(siderealTime - ra, -12.0, 12.0);
}

static wxString hour_angle_log_value(double ra, double siderealTime)
{
    if (valid_sidereal_time(siderealTime) && valid_ra(ra))
        return wxString::Format("%.6f", hour_angle(ra, siderealTime));
    return "N/A";
}

static const char *pier_side_name(PierSide side)
{
    switch (side)
    {
    case PIER_SIDE_EAST:
        return "east";
    case PIER_SIDE_WEST:
        return "west";
    default:
        return "unknown";
    }
}

static void get_paused(JObj& response, const json_value *params)
{
    VERIFY_GUIDER(response);
    response << jrpc_result(pFrame->pGuider->IsPaused());
}

static void set_paused(JObj& response, const json_value *params)
{
    Params p("paused", "type", params);
    const json_value *jv = p.param("paused");

    bool val;
    if (!jv || !bool_param(jv, &val))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected bool param at index 0");
        return;
    }

    PauseType pause = PAUSE_NONE;

    if (val)
    {
        pause = PAUSE_GUIDING;

        jv = p.param("type");
        if (jv)
        {
            if (jv->type == JSON_STRING)
            {
                if (strcmp(jv->string_value, "full") == 0)
                    pause = PAUSE_FULL;
            }
            else
            {
                response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected string param at index 1");
                return;
            }
        }
    }

    pFrame->SetPaused(pause);

    response << jrpc_result(0);
}

static void loop(JObj& response, const json_value *params)
{
    bool error = pFrame->StartLooping();

    if (error)
        response << jrpc_error(1, "could not start looping");
    else
        response << jrpc_result(0);
}

static void stop_capture(JObj& response, const json_value *params)
{
    pFrame->StopCapturing();
    response << jrpc_result(0);
}

static bool parse_rect(wxRect *r, const json_value *j)
{
    if (j->type != JSON_ARRAY)
        return false;

    int a[4];
    const json_value *jv = j->first_child;
    for (int i = 0; i < 4; i++)
    {
        if (!jv || jv->type != JSON_INT)
            return false;
        a[i] = jv->int_value;
        jv = jv->next_sibling;
    }
    if (jv)
        return false; // extra value

    r->x = a[0];
    r->y = a[1];
    r->width = a[2];
    r->height = a[3];

    return true;
}

static void find_star(JObj& response, const json_value *params)
{
    VERIFY_GUIDER(response);

    Params p("roi", params);

    wxRect roi;
    const json_value *j = p.param("roi");
    if (j && !parse_rect(&roi, j))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "invalid ROI param");
        return;
    }

    bool error = pFrame->AutoSelectStar(roi);

    if (!error)
    {
        const PHD_Point& lockPos = pFrame->pGuider->LockPosition();
        if (lockPos.IsValid())
        {
            response << jrpc_result(lockPos);
            return;
        }
    }

    response << jrpc_error(1, "could not find star");
}

static void get_pixel_scale(JObj& response, const json_value *params)
{
    double scale = pFrame->GetCameraPixelScale();
    if (scale == 1.0)
        response << jrpc_result(NULL_VALUE); // scale unknown
    else
        response << jrpc_result(scale);
}

static void get_app_state(JObj& response, const json_value *params)
{
    EXPOSED_STATE st = Guider::GetExposedState();
    response << jrpc_result(state_name(st));
}

static void get_lock_position(JObj& response, const json_value *params)
{
    VERIFY_GUIDER(response);

    const PHD_Point& lockPos = pFrame->pGuider->LockPosition();
    if (lockPos.IsValid())
        response << jrpc_result(lockPos);
    else
        response << jrpc_result(NULL_VALUE);
}

// {"method": "set_lock_position", "params": [X, Y, true], "id": 1}
// Optional 4th param `find_star` (boolean, default false): only meaningful when
// `exact` is true. When set, PHD2 additionally runs Star::Find near the new lock and
// refreshes m_primaryStar / CurrentPosition. This lets a client get both the exact pixel
// lock and a consistent star/lock pairing in a single call (previously two calls were
// needed: exact=false to refresh the current star, then exact=true to override the lock).
static void set_lock_position(JObj& response, const json_value *params)
{
    Params p("x", "y", "exact", "find_star", params);
    const json_value *p0 = p.param("x"), *p1 = p.param("y");
    double x, y;

    if (!p0 || !p1 || !float_param(p0, &x) || !float_param(p1, &y))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected lock position x, y params");
        return;
    }

    bool exact = true;
    const json_value *p2 = p.param("exact");

    if (p2)
    {
        if (!bool_param(p2, &exact))
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected boolean param at index 2");
            return;
        }
    }

    bool findStar = false;
    const json_value *p3 = p.param("find_star");

    if (p3)
    {
        if (!bool_param(p3, &findStar))
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected boolean find_star param");
            return;
        }
    }

    VERIFY_GUIDER(response);

    bool error;

    if (exact)
    {
        if (findStar)
            error = pFrame->pGuider->SetLockPositionAndRefreshStar(PHD_Point(x, y));
        else
            error = pFrame->pGuider->SetLockPosition(PHD_Point(x, y));
    }
    else
    {
        error = pFrame->pGuider->SetLockPosToStarAtPosition(PHD_Point(x, y));
    }

    if (error)
    {
        response << jrpc_error(JSONRPC_INVALID_REQUEST, "could not set lock position");
        return;
    }

    response << jrpc_result(0);
}

inline static const char *string_val(const json_value *j)
{
    return j->type == JSON_STRING ? j->string_value : "";
}

enum WHICH_MOUNT
{
    MOUNT,
    AO,
    WHICH_MOUNT_BOTH,
    WHICH_MOUNT_ERR
};

static WHICH_MOUNT which_mount(const json_value *p)
{
    WHICH_MOUNT r = MOUNT;
    if (p)
    {
        r = WHICH_MOUNT_ERR;
        if (p->type == JSON_STRING)
        {
            if (wxStricmp(p->string_value, "ao") == 0)
                r = AO;
            else if (wxStricmp(p->string_value, "mount") == 0)
                r = MOUNT;
            else if (wxStricmp(p->string_value, "both") == 0)
                r = WHICH_MOUNT_BOTH;
        }
    }
    return r;
}

static void clear_calibration(JObj& response, const json_value *params)
{
    bool clear_mount;
    bool clear_ao;

    if (!params)
    {
        clear_mount = clear_ao = true;
    }
    else
    {
        Params p("which", params);

        clear_mount = clear_ao = false;

        WHICH_MOUNT which = which_mount(p.param("which"));
        switch (which)
        {
        case MOUNT:
            clear_mount = true;
            break;
        case AO:
            clear_ao = true;
            break;
        case WHICH_MOUNT_BOTH:
            clear_mount = clear_ao = true;
            break;
        case WHICH_MOUNT_ERR:
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected param \"mount\", \"ao\", or \"both\"");
            return;
        }
    }

    Mount *mount = TheScope();
    Mount *ao = TheAO();

    if (mount && clear_mount)
        mount->ClearCalibration();

    if (ao && clear_ao)
        ao->ClearCalibration();

    response << jrpc_result(0);
}

static void flip_calibration(JObj& response, const json_value *params)
{
    bool error = pFrame->FlipCalibrationData();

    if (error)
        response << jrpc_error(1, "could not flip calibration");
    else
        response << jrpc_result(0);
}

static void get_lock_shift_enabled(JObj& response, const json_value *params)
{
    VERIFY_GUIDER(response);
    bool enabled = pFrame->pGuider->GetLockPosShiftParams().shiftEnabled;
    response << jrpc_result(enabled);
}

static void set_lock_shift_enabled(JObj& response, const json_value *params)
{
    Params p("enabled", params);
    const json_value *val = p.param("enabled");
    bool enable;
    if (!val || !bool_param(val, &enable))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected enabled boolean param");
        return;
    }

    VERIFY_GUIDER(response);

    pFrame->pGuider->EnableLockPosShift(enable);

    response << jrpc_result(0);
}

static bool is_camera_shift_req(const json_value *params)
{
    Params p("axes", params);
    const json_value *j = p.param("axes");
    if (j)
    {
        const char *axes = string_val(j);
        if (wxStricmp(axes, "x/y") == 0 || wxStricmp(axes, "camera") == 0)
        {
            return true;
        }
    }
    return false;
}

static JObj& operator<<(JObj& j, const LockPosShiftParams& l)
{
    j << NV("enabled", l.shiftEnabled);
    if (l.shiftRate.IsValid())
    {
        j << NV("rate", l.shiftRate) << NV("units", l.shiftUnits == UNIT_ARCSEC ? "arcsec/hr" : "pixels/hr")
          << NV("axes", l.shiftIsMountCoords ? "RA/Dec" : "X/Y");
    }
    return j;
}

static void get_lock_shift_params(JObj& response, const json_value *params)
{
    VERIFY_GUIDER(response);

    const LockPosShiftParams& lockShift = pFrame->pGuider->GetLockPosShiftParams();
    JObj rslt;

    if (is_camera_shift_req(params))
    {
        LockPosShiftParams tmp;
        tmp.shiftEnabled = lockShift.shiftEnabled;
        const ShiftPoint& lock = pFrame->pGuider->LockPosition();
        tmp.shiftRate = lock.ShiftRate() * 3600; // px/sec => px/hr
        tmp.shiftUnits = UNIT_PIXELS;
        tmp.shiftIsMountCoords = false;
        tmp.decValid = lockShift.decValid;
        tmp.dec = lockShift.dec;
        rslt << tmp;
    }
    else
        rslt << lockShift;

    response << jrpc_result(rslt);
}

static bool get_double(double *d, const json_value *j)
{
    if (j->type == JSON_FLOAT)
    {
        *d = j->float_value;
        return true;
    }
    else if (j->type == JSON_INT)
    {
        *d = j->int_value;
        return true;
    }
    return false;
}

static bool parse_point(PHD_Point *pt, const json_value *j)
{
    if (j->type != JSON_ARRAY)
        return false;
    const json_value *jx = j->first_child;
    if (!jx)
        return false;
    const json_value *jy = jx->next_sibling;
    if (!jy || jy->next_sibling)
        return false;
    double x, y;
    if (!get_double(&x, jx) || !get_double(&y, jy))
        return false;
    pt->SetXY(x, y);
    return true;
}

static bool parse_lock_shift_params(LockPosShiftParams *shift, const json_value *params, wxString *error)
{
    // "params":[{"rate":[3.3,1.1],"units":"arcsec/hr","axes":"RA/Dec"}]
    // or
    // "params":{"rate":[3.3,1.1],"units":"arcsec/hr","axes":"RA/Dec"}

    if (params && params->type == JSON_ARRAY)
        params = params->first_child;

    Params p("rate", "units", "axes", "dec", params);

    shift->shiftUnits = UNIT_ARCSEC;
    shift->shiftIsMountCoords = true;

    const json_value *j;

    j = p.param("rate");
    if (!j || !parse_point(&shift->shiftRate, j))
    {
        *error = "expected rate value array";
        return false;
    }

    j = p.param("units");
    const char *units = j ? string_val(j) : "";

    if (wxStricmp(units, "arcsec/hr") == 0 || wxStricmp(units, "arc-sec/hr") == 0)
    {
        shift->shiftUnits = UNIT_ARCSEC;
    }
    else if (wxStricmp(units, "pixels/hr") == 0)
    {
        shift->shiftUnits = UNIT_PIXELS;
    }
    else
    {
        *error = "expected units 'arcsec/hr' or 'pixels/hr'";
        return false;
    }

    j = p.param("axes");
    const char *axes = j ? string_val(j) : "";

    if (wxStricmp(axes, "RA/Dec") == 0)
    {
        shift->shiftIsMountCoords = true;
    }
    else if (wxStricmp(axes, "X/Y") == 0)
    {
        shift->shiftIsMountCoords = false;
    }
    else
    {
        *error = "expected axes 'RA/Dec' or 'X/Y'";
        return false;
    }

    j = p.param("dec");
    double dec;
    shift->decValid = false;
    if (j && float_param(j, &dec))
    {
        shift->dec = dec;
        shift->decValid = true;
    }

    return true;
}

static void set_lock_shift_params(JObj& response, const json_value *params)
{
    wxString err;
    LockPosShiftParams shift;
    if (!parse_lock_shift_params(&shift, params, &err))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, err);
        return;
    }

    VERIFY_GUIDER(response);

    pFrame->pGuider->SetLockPosShiftRate(shift, true);

    response << jrpc_result(0);
}

static void save_image(JObj& response, const json_value *params)
{
    VERIFY_GUIDER(response);

    if (!pFrame->pGuider->CurrentImage()->ImageData)
    {
        response << jrpc_error(2, "no image available");
        return;
    }

    wxString fname = wxFileName::CreateTempFileName(MyFrame::GetDefaultFileDir() + PATHSEPSTR + "save_image_");

    if (pFrame->pGuider->SaveCurrentImage(fname))
    {
        ::wxRemove(fname);
        response << jrpc_error(3, "error saving image");
        return;
    }

    JObj rslt;
    rslt << NV("filename", fname);
    response << jrpc_result(rslt);
}

static bool IsValidBinning(GuideCamera *camera, int binning, wxString *message)
{
    auto choices = camera->GetBinningChoices();
    if (choices.find(binning) != choices.end())
        return true;
    wxString buf = wxString::Format("Invalid binning value (%d). Valid choices are: ", binning);
    bool first = true;
    for (auto choice : choices)
    {
        if (first)
            first = false;
        else
            buf.Append(", ");
        buf.Append(wxString::Format("%d", choice.first));
    }
    *message = buf;
    return false;
}

static void capture_single_frame(JObj& response, const json_value *params)
{
    if (pFrame->CaptureActive)
    {
        response << jrpc_error(1, "cannot capture single frame when capture is currently active");
        return;
    }
    if (!pCamera || !pCamera->Connected)
    {
        response << jrpc_error(1, "cannot capture single frame when camera is not connected");
    }

    Params p("exposure", "binning", "gain", "subframe", "path", "save", params);

    const json_value *j;

    int exposure = pFrame->RequestedExposureDuration();
    if ((j = p.param("exposure")) != nullptr)
    {
        if (j->type != JSON_INT || j->int_value < 1 || j->int_value > 10 * 60000)
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected exposure param");
            return;
        }
        exposure = j->int_value;
    }

    wxByte binning = pCamera->GetBinning();
    if ((j = p.param("binning")) != nullptr)
    {
        if (j->type != JSON_INT)
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "binning value must be an integer");
            return;
        }
        wxString message;
        if (!IsValidBinning(pCamera, j->int_value, &message))
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, message);
            return;
        }
        binning = j->int_value;
    }

    int gain = pCamera->GetCameraGain();
    if ((j = p.param("gain")) != nullptr)
    {
        if (j->type != JSON_INT || j->int_value < 0 || j->int_value > 100)
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "invalid gain value: must be between 0 and 100");
            return;
        }
        gain = j->int_value;
    }

    wxRect subframe;
    if ((j = p.param("subframe")) != nullptr)
        if (!parse_rect(&subframe, j))
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "invalid subframe param");
            return;
        }

    wxString path;
    if ((j = p.param("path")) != nullptr)
    {
        if (j->type != JSON_STRING)
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "invalid path param: string expected");
            return;
        }
        wxFileName fn(j->string_value);
        if (!fn.IsAbsolute())
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "path param must be an absolute path");
            return;
        }
        if (fn.Exists())
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "destination file already exists");
            return;
        }
        path = j->string_value;
    }

    bool save = !path.empty();
    if ((j = p.param("save")) != nullptr)
    {
        if (!bool_param(j, &save))
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "save param must be a boolean");
            return;
        }
    }

    if (!save && !path.empty())
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "path param not allowed when save = false");
        return;
    }

    bool err = pFrame->StartSingleExposure(exposure, binning, gain, subframe, save, path);
    if (err)
    {
        response << jrpc_error(2, "failed to start exposure");
        return;
    }

    response << jrpc_result(0);
}

static void get_use_subframes(JObj& response, const json_value *params)
{
    response << jrpc_result(pCamera && pCamera->UseSubframes);
}

static void get_search_region(JObj& response, const json_value *params)
{
    VERIFY_GUIDER(response);
    response << jrpc_result(pFrame->pGuider->GetSearchRegion());
}

struct B64Encode
{
    static const char *const E;
    std::ostringstream os;
    unsigned int t;
    size_t nread;

    B64Encode() : t(0), nread(0) { }
    void append1(unsigned char ch)
    {
        t <<= 8;
        t |= ch;
        if (++nread % 3 == 0)
        {
            os << E[t >> 18] << E[(t >> 12) & 0x3F] << E[(t >> 6) & 0x3F] << E[t & 0x3F];
            t = 0;
        }
    }
    void append(const void *src_, size_t len)
    {
        const unsigned char *src = (const unsigned char *) src_;
        const unsigned char *const end = src + len;
        while (src < end)
            append1(*src++);
    }
    std::string finish()
    {
        switch (nread % 3)
        {
        case 1:
            os << E[t >> 2] << E[(t & 0x3) << 4] << "==";
            break;
        case 2:
            os << E[t >> 10] << E[(t >> 4) & 0x3F] << E[(t & 0xf) << 2] << '=';
            break;
        }
        return os.str();
    }
};
const char *const B64Encode::E = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void get_star_image(JObj& response, const json_value *params)
{
    int reqsize = 15;
    Params p("size", params);
    const json_value *val = p.param("size");
    if (val)
    {
        if (val->type != JSON_INT || (reqsize = val->int_value) < 15)
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "invalid image size param");
            return;
        }
    }

    VERIFY_GUIDER(response);

    Guider *guider = pFrame->pGuider;
    const usImage *img = guider->CurrentImage();
    const PHD_Point& star = guider->CurrentPosition();

    if (guider->GetState() < GUIDER_STATE::STATE_SELECTED || !img->ImageData || !star.IsValid())
    {
        response << jrpc_error(2, "no star selected");
        return;
    }

    int const halfw = wxMin((reqsize - 1) / 2, 31);
    int const fullw = 2 * halfw + 1;
    int const sx = (int) rint(star.X);
    int const sy = (int) rint(star.Y);
    wxRect rect(sx - halfw, sy - halfw, fullw, fullw);
    if (img->Subframe.IsEmpty())
        rect.Intersect(wxRect(img->Size));
    else
        rect.Intersect(img->Subframe);

    B64Encode enc;
    for (int y = rect.GetTop(); y <= rect.GetBottom(); y++)
    {
        const unsigned short *p = img->ImageData + y * img->Size.GetWidth() + rect.GetLeft();
        enc.append(p, rect.GetWidth() * sizeof(unsigned short));
    }

    PHD_Point pos(star);
    pos.X -= rect.GetLeft();
    pos.Y -= rect.GetTop();

    JObj rslt;
    rslt << NV("frame", img->FrameNum) << NV("width", rect.GetWidth()) << NV("height", rect.GetHeight()) << NV("star_pos", pos)
         << NV("pixels", enc.finish());

    response << jrpc_result(rslt);
}

static bool parse_settle(SettleParams *settle, const json_value *j, wxString *error)
{
    bool found_pixels = false, found_time = false, found_timeout = false;

    json_for_each(t, j)
    {
        if (float_param("pixels", t, &settle->tolerancePx))
        {
            found_pixels = true;
            continue;
        }
        double d;
        if (float_param("time", t, &d))
        {
            settle->settleTimeSec = (int) floor(d);
            found_time = true;
            continue;
        }
        if (float_param("timeout", t, &d))
        {
            settle->timeoutSec = (int) floor(d);
            found_timeout = true;
            continue;
        }
    }

    settle->frames = 99999;

    bool ok = found_pixels && found_time && found_timeout;
    if (!ok)
        *error = "invalid settle params";

    return ok;
}

static void guide(JObj& response, const json_value *params)
{
    // params:
    //   settle [object]:
    //     pixels [float]
    //     arcsecs [float]
    //     frames [integer]
    //     time [integer]
    //     timeout [integer]
    //   recalibrate: boolean
    //
    // {"method": "guide", "params": [{"pixels": 0.5, "time": 6, "timeout": 30}, false], "id": 42}
    //    or
    // {"method": "guide", "params": {"settle": {"pixels": 0.5, "time": 6, "timeout": 30}, "recalibrate": false}, "id": 42}
    //
    // todo:
    //   accept tolerance in arcsec or pixels
    //   accept settle time in seconds or frames

    SettleParams settle;

    Params p("settle", "recalibrate", "roi", params);
    const json_value *p0 = p.param("settle");
    if (!p0 || p0->type != JSON_OBJECT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected settle object param");
        return;
    }
    wxString errMsg;
    if (!parse_settle(&settle, p0, &errMsg))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, errMsg);
        return;
    }

    bool recalibrate = false;
    const json_value *p1 = p.param("recalibrate");
    if (p1)
    {
        if (!bool_param(p1, &recalibrate))
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected bool value for recalibrate");
            return;
        }
    }

    wxRect roi;
    const json_value *p2 = p.param("roi");
    if (p2 && !parse_rect(&roi, p2))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "invalid ROI param");
        return;
    }

    if (recalibrate && !pConfig->Global.GetBoolean("/server/guide_allow_recalibrate", true))
    {
        Debug.AddLine("ignoring client recalibration request since guide_allow_recalibrate = false");
        recalibrate = false;
    }

    wxString err;
    unsigned int ctrlOptions = GUIDEOPT_USE_STICKY_LOCK;
    if (recalibrate)
        ctrlOptions |= GUIDEOPT_FORCE_RECAL;
    if (!PhdController::CanGuide(&err))
        response << jrpc_error(1, err);
    else if (PhdController::Guide(ctrlOptions, settle, roi, &err))
        response << jrpc_result(0);
    else
        response << jrpc_error(1, err);
}

static void dither(JObj& response, const json_value *params)
{
    // params:
    //   amount [integer] - max pixels to move in each axis
    //   raOnly [bool] - when true, only dither ra
    //   settle [object]:
    //     pixels [float]
    //     arcsecs [float]
    //     frames [integer]
    //     time [integer]
    //     timeout [integer]
    //
    // {"method": "dither", "params": [10, false, {"pixels": 1.5, "time": 8, "timeout": 30}], "id": 42}
    //    or
    // {"method": "dither", "params": {"amount": 10, "raOnly": false, "settle": {"pixels": 1.5, "time": 8, "timeout": 30}},
    // "id": 42}

    Params p("amount", "raOnly", "settle", params);
    const json_value *jv;
    double ditherAmt;

    jv = p.param("amount");
    if (!jv || !float_param(jv, &ditherAmt))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected dither amount param");
        return;
    }

    bool raOnly = false;
    jv = p.param("raOnly");
    if (jv)
    {
        if (!bool_param(jv, &raOnly))
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected dither raOnly param");
            return;
        }
    }

    SettleParams settle;

    jv = p.param("settle");
    if (!jv || jv->type != JSON_OBJECT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected settle object param");
        return;
    }
    wxString errMsg;
    if (!parse_settle(&settle, jv, &errMsg))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, errMsg);
        return;
    }

    wxString error;
    if (PhdController::Dither(fabs(ditherAmt), raOnly, settle, &error))
        response << jrpc_result(0);
    else
        response << jrpc_error(1, error);
}

static void shutdown(JObj& response, const json_value *params)
{
    wxGetApp().TerminateApp();

    response << jrpc_result(0);
}

static void get_camera_binning(JObj& response, const json_value *params)
{
    if (pCamera && pCamera->Connected)
    {
        int binning = pCamera->GetBinning();
        response << jrpc_result(binning);
    }
    else
        response << jrpc_error(1, "camera not connected");
}

static void get_camera_frame_size(JObj& response, const json_value *params)
{
    if (pCamera && pCamera->Connected)
    {
        response << jrpc_result(pCamera->FrameSize);
    }
    else
        response << jrpc_error(1, "camera not connected");
}

static void get_guide_output_enabled(JObj& response, const json_value *params)
{
    if (pMount)
        response << jrpc_result(pMount->GetGuidingEnabled());
    else
        response << jrpc_error(1, "mount not defined");
}

static void set_guide_output_enabled(JObj& response, const json_value *params)
{
    Params p("enabled", params);
    const json_value *val = p.param("enabled");
    bool enable;
    if (!val || !bool_param(val, &enable))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected enabled boolean param");
        return;
    }

    if (pMount)
    {
        pMount->SetGuidingEnabled(enable);
        response << jrpc_result(0);
    }
    else
        response << jrpc_error(1, "mount not defined");
}

static bool axis_param(const Params& p, GuideAxis *a)
{
    const json_value *val = p.param("axis");
    if (!val || val->type != JSON_STRING)
        return false;

    bool ok = true;

    if (wxStricmp(val->string_value, "ra") == 0)
        *a = GUIDE_RA;
    else if (wxStricmp(val->string_value, "x") == 0)
        *a = GUIDE_X;
    else if (wxStricmp(val->string_value, "dec") == 0)
        *a = GUIDE_DEC;
    else if (wxStricmp(val->string_value, "y") == 0)
        *a = GUIDE_Y;
    else
        ok = false;

    return ok;
}

static void get_algo_param_names(JObj& response, const json_value *params)
{
    Params p("axis", params);
    GuideAxis a;
    if (!axis_param(p, &a))
    {
        response << jrpc_error(1, "expected axis name param");
        return;
    }
    wxArrayString ary;
    ary.push_back("algorithmName");

    if (pMount)
    {
        GuideAlgorithm *alg = a == GUIDE_X ? pMount->GetXGuideAlgorithm() : pMount->GetYGuideAlgorithm();
        alg->GetParamNames(ary);
    }

    JAry names;
    for (auto it = ary.begin(); it != ary.end(); ++it)
        names << ('"' + json_escape(*it) + '"');

    response << jrpc_result(names);
}

static void get_algo_param(JObj& response, const json_value *params)
{
    Params p("axis", "name", params);
    GuideAxis a;
    if (!axis_param(p, &a))
    {
        response << jrpc_error(1, "expected axis name param");
        return;
    }
    const json_value *name = p.param("name");
    if (!name || name->type != JSON_STRING)
    {
        response << jrpc_error(1, "expected param name param");
        return;
    }
    bool ok = false;
    double val;
    if (pMount)
    {
        GuideAlgorithm *alg = a == GUIDE_X ? pMount->GetXGuideAlgorithm() : pMount->GetYGuideAlgorithm();
        if (strcmp(name->string_value, "algorithmName") == 0)
        {
            response << jrpc_result(alg->GetGuideAlgorithmClassName());
            return;
        }
        ok = alg->GetParam(name->string_value, &val);
    }
    if (ok)
        response << jrpc_result(val);
    else
        response << jrpc_error(1, "could not get param");
}

static void set_algo_param(JObj& response, const json_value *params)
{
    Params p("axis", "name", "value", params);
    GuideAxis a;
    if (!axis_param(p, &a))
    {
        response << jrpc_error(1, "expected axis name param");
        return;
    }
    const json_value *name = p.param("name");
    if (!name || name->type != JSON_STRING)
    {
        response << jrpc_error(1, "expected param name param");
        return;
    }
    const json_value *val = p.param("value");
    double v;
    if (!float_param(val, &v))
    {
        response << jrpc_error(1, "expected param value param");
        return;
    }
    bool ok = false;
    if (pMount)
    {
        GuideAlgorithm *alg = a == GUIDE_X ? pMount->GetXGuideAlgorithm() : pMount->GetYGuideAlgorithm();
        ok = alg->SetParam(name->string_value, v);
    }
    if (ok)
    {
        response << jrpc_result(0);
        if (pFrame->pGraphLog)
            pFrame->pGraphLog->UpdateControls();
    }
    else
        response << jrpc_error(1, "could not set param");
}

static void get_dec_guide_mode(JObj& response, const json_value *params)
{
    Scope *scope = TheScope();
    DEC_GUIDE_MODE mode = scope ? scope->GetDecGuideMode() : DEC_NONE;
    wxString s = Scope::DecGuideModeStr(mode);
    response << jrpc_result(s);
}

static void set_dec_guide_mode(JObj& response, const json_value *params)
{
    Params p("mode", params);
    const json_value *mode = p.param("mode");
    if (!mode || mode->type != JSON_STRING)
    {
        response << jrpc_error(1, "expected mode param");
        return;
    }
    DEC_GUIDE_MODE m = DEC_AUTO;
    bool found = false;
    for (int im = DEC_NONE; im <= DEC_SOUTH; im++)
    {
        m = (DEC_GUIDE_MODE) im;
        if (wxStricmp(mode->string_value, Scope::DecGuideModeStr(m)) == 0)
        {
            found = true;
            break;
        }
    }
    if (!found)
    {
        response << jrpc_error(1, "invalid dec guide mode param");
        return;
    }

    Scope *scope = TheScope();
    if (scope)
        scope->SetDecGuideMode(m);

    if (pFrame->pGraphLog)
        pFrame->pGraphLog->UpdateControls();

    response << jrpc_result(0);
}

static void get_settling(JObj& response, const json_value *params)
{
    bool settling = PhdController::IsSettling();
    response << jrpc_result(settling);
}

static void get_variable_delay_settings(JObj& response, const json_value *params)
{
    JObj rslt;

    VarDelayCfg delayParams = pFrame->GetVariableDelayConfig();
    rslt << NV("Enabled", delayParams.enabled) << NV("ShortDelaySeconds", delayParams.shortDelay / 1000)
         << NV("LongDelaySeconds", delayParams.longDelay / 1000);
    response << jrpc_result(rslt);
}

static void get_guiding_period(JObj& response, const json_value *params)
{
    JObj rslt;
    int exposure;
    int timeLapse;
    rslt << NV("period", pFrame->GetGuidingPeriod(&exposure, &timeLapse));
    rslt << NV("exposure", exposure);
    rslt << NV("timelapse", timeLapse);
    response << jrpc_result(rslt);
}

static void set_time_lapse(JObj& response, const json_value *params)
{
    Params p("interval", params);
    const json_value *val = p.param("interval");
    if (!val || val->type != JSON_INT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected interval param");
        return;
    }
    if (pFrame->pGuider)
        pFrame->pGuider->m_SolarSystemObject.SetTimeLapse(val->int_value);
    response << jrpc_result(0);
}

static void get_pixel_size(JObj& response, const json_value *params)
{
    if (pCamera)
    {
        double pixelSize = pCamera->GetCameraPixelSize();
        response << jrpc_result(pixelSize);
    }
    else
    {
        response << jrpc_error(1, "camera not connected");
    }
}

static void set_pixel_size(JObj& response, const json_value *params)
{
    Params p("PixelSize", params);
    const json_value *val = p.param("PixelSize");
    double pixelSize;
    if (!val || !float_param(val, &pixelSize))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected pixel size param");
        return;
    }
    if (pCamera)
        pCamera->SetCameraPixelSize(pixelSize);
    response << jrpc_result(0);
}

static void get_focal_length(JObj& response, const json_value *params)
{
    int focalLength = pFrame->GetFocalLength();
    response << jrpc_result(focalLength);
}

static void set_focal_length(JObj& response, const json_value *params)
{
    Params p("length", params);
    const json_value *val = p.param("length");
    if (!val || val->type != JSON_INT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected focal length param");
        return;
    }
    int focalLength = val->int_value;
    if (focalLength < 50)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "focal length must be at least 50 mm");
        return;
    }
    pFrame->SetFocalLength(focalLength);
    response << jrpc_result(0);
}

static void set_planetary_mode(JObj& response, const json_value *params)
{
    Params p("mode", params);
    const json_value *val = p.param("mode");
    bool planetaryMode;
    if (!val || !bool_param(val, &planetaryMode))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected planetary mode param");
        return;
    }
    if (pFrame->pGuider)
        pFrame->pGuider->m_SolarSystemObject.Set_SolarSystemObjMode(planetaryMode);
    response << jrpc_result(0);
}

static void set_guide_frame(JObj& response, const json_value *params)
{
    Params p("path", params);
    const json_value *path = p.param("path");

    if (!path || path->type != JSON_STRING)
    {
        response << jrpc_error(1, "expected path string");
        return;
    }
    if (pCamera)
        pCamera->SetProperty("path_broadcast", path->string_value);
}

static void get_guide_frame(JObj& response, const json_value *params)
{
    VERIFY_GUIDER(response);

    Guider *guider = pFrame->pGuider;
    const usImage *img = guider->CurrentImage();

    if (!img->ImageData)
    {
        response << jrpc_error(2, "no image available");
        return;
    }

    const int width = img->Size.GetWidth();
    const int height = img->Size.GetHeight();
    const int LIMIT_PX = 2048;
    double scale = 1.0;
    int outWidth = width;
    int outHeight = height;
    cv::Mat decimated;

    if (wxMax(width, height) > LIMIT_PX)
    {
        scale = static_cast<double>(wxMax(width, height)) / LIMIT_PX;
        outWidth = cvRound(width / scale);
        outHeight = cvRound(height / scale);
        cv::Mat src(height, width, CV_16UC1, img->ImageData);
        cv::resize(src, decimated, cv::Size(outWidth, outHeight), 0, 0, cv::INTER_LINEAR);
    }

    B64Encode enc;
    for (int y = 0; y < outHeight; y++)
    {
        // decimated.empty() when no resize happened - encode straight from the usImage.
        // Otherwise use Mat::ptr<> so any per-row alignment padding is respected.
        const unsigned short *row = decimated.empty()
            ? (img->ImageData + y * width)
            : decimated.ptr<unsigned short>(y);
        enc.append(row, outWidth * sizeof(unsigned short));
    }

    JObj rslt;
    rslt << NV("frame", img->FrameNum)
         << NV("width", outWidth)
         << NV("height", outHeight)
         << NV("scale", scale, 3)
         << NV("pixels", enc.finish());

    response << jrpc_result(rslt);
}

static void get_cal_settings(JObj& response, const json_value *params)
{
    JObj rslt;
    double guideSpeed = 0.5;
    double declination = 0;
    if (pFrame->pGuider)
        pFrame->pGuider->m_SolarSystemObject.GetCalSettings(&declination, &guideSpeed);
    rslt << NV("speed", guideSpeed) << NV("dec", declination);
    response << jrpc_result(rslt);
}

static void get_equatorial_system(JObj& response, const json_value *params)
{
    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    int system = 0;
    if (!pPointingSource->GetEquatorialSystem(&system))
        response << jrpc_result(system);
    else
        response << jrpc_error(1, "failed to get equatorial system");
}

static JAry axis_rates_json(const std::vector<Scope::AxisRate>& rates)
{
    JAry ary;
    for (const auto& rate : rates)
    {
        JObj item;
        item << NV("Minimum", rate.minimum) << NV("Maximum", rate.maximum);
        ary << item;
    }
    return ary;
}

static bool axis_rate_supported(const std::vector<Scope::AxisRate>& rates, double rate, double *remappedRate)
{
    double absRate = std::fabs(rate);
    if (remappedRate)
        *remappedRate = rate;
    if (absRate == 0.0)
        return true;

    bool supported = false;
    double bestAbsRate = absRate;
    double bestDiff = 0.0;
    for (const auto& supportedRate : rates)
    {
        double minimum = wxMin(supportedRate.minimum, supportedRate.maximum);
        double maximum = wxMax(supportedRate.minimum, supportedRate.maximum);
        double candidateAbsRate = absRate;
        bool rateMatches = false;

        if (absRate >= minimum && absRate <= maximum)
        {
            rateMatches = true;
        }
        else if (absRate < minimum)
        {
            double tolerance = wxMax(1e-12, minimum * 0.001);
            if (minimum - absRate <= tolerance)
            {
                candidateAbsRate = minimum;
                rateMatches = true;
            }
        }
        else
        {
            double tolerance = wxMax(1e-12, maximum * 0.001);
            if (absRate - maximum <= tolerance)
            {
                candidateAbsRate = maximum;
                rateMatches = true;
            }
        }

        if (rateMatches)
        {
            double diff = std::fabs(candidateAbsRate - absRate);
            if (!supported || diff < bestDiff)
            {
                supported = true;
                bestAbsRate = candidateAbsRate;
                bestDiff = diff;
            }
        }
    }

    if (!supported)
        return false;

    if (remappedRate)
        *remappedRate = std::signbit(rate) ? -bestAbsRate : bestAbsRate;
    return true;
}

static wxString axis_rate_ranges_string(const std::vector<Scope::AxisRate>& rates)
{
    if (rates.empty())
        return "none";

    wxString ranges;
    for (size_t i = 0; i < rates.size(); ++i)
    {
        if (!ranges.empty())
            ranges += ", ";
        ranges += wxString::Format("[%.9g, %.9g]", rates[i].minimum, rates[i].maximum);
    }

    return ranges;
}

static JAry strings_json(const std::vector<wxString>& strings)
{
    JAry ary;
    for (const auto& str : strings)
        ary << str;
    return ary;
}

static void get_mount_caps(JObj& response, const json_value* params)
{
    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    int system = 0;
    bool async = false;
    if (!pPointingSource->GetEquatorialSystem(&system))
        response << NV("EquatorialSystem", system);
    bool canMoveAxisRA = pPointingSource->CanMoveAxis(GUIDE_RA);
    bool canMoveAxisDec = pPointingSource->CanMoveAxis(GUIDE_DEC);
    response << NV("CanSlewAsync", pPointingSource->CanSlewAsync());
    response << NV("CanSlew", pPointingSource->CanSlew());
    response << NV("CanUnpark", pPointingSource->CanUnpark());
    response << NV("CanCheckSlewing", pPointingSource->CanCheckSlewing());
    response << NV("CanSetRightAscensionRate", pPointingSource->CanSetRightAscensionRate());
    response << NV("CanSetDeclinationRate", pPointingSource->CanSetDeclinationRate());
    response << NV("CanMoveAxis", canMoveAxisRA || canMoveAxisDec);
    response << NV("CanMoveAxisRA", canMoveAxisRA);
    response << NV("CanMoveAxisDec", canMoveAxisDec);
    response << NV("CanSync", pPointingSource->CanSync());
    std::vector<Scope::AxisRate> axisRates;
    if (!pPointingSource->GetAxisRates(GUIDE_RA, &axisRates))
    {
        JAry ary = axis_rates_json(axisRates);
        response << NV("AxisRatesRA", ary);
    }
    if (!pPointingSource->GetAxisRates(GUIDE_DEC, &axisRates))
    {
        JAry ary = axis_rates_json(axisRates);
        response << NV("AxisRatesDec", ary);
    }
    bool refraction = false;
    if (!pPointingSource->DoesRefraction(&refraction))
        response << NV("DoesRefraction", refraction);
    response << jrpc_result(0);
}

static void does_refraction(JObj& response, const json_value *params)
{
    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    bool refraction = false;
    if (!pPointingSource->DoesRefraction(&refraction))
        response << jrpc_result(refraction);
    else
        response << jrpc_error(1, "failed to get refraction");
}

static void is_parked(JObj& response, const json_value *params)
{
    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    bool parked = false;
    if (!pPointingSource->IsParked(&parked))
        response << jrpc_result(parked);
    else
        response << jrpc_error(1, "failed to get parked state");
}

static void park(JObj& response, const json_value *params)
{
    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }
    if (!pPointingSource->Park())
    {
        // Refresh the cached parked state so the next guide pulse / slew / move-axis is gated
        // by Mount::IsKnownParked() without a separate poll.
        pPointingSource->SetLastKnownParked(true);
        response << jrpc_result(0);
    }
    else
        response << jrpc_error(1, "failed to park mount");
}

static void unpark(JObj& response, const json_value *params)
{
    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }
    if (!pPointingSource->Unpark())
    {
        pPointingSource->SetLastKnownParked(false);
        response << jrpc_result(0);
    }
    else
        response << jrpc_error(1, "failed to park mount");
}

static void slew_to_coordinates(JObj& response, const json_value* params)
{
    Params p("ra", "dec", params);
    const json_value *raVal = p.param("ra");
    const json_value *decVal = p.param("dec");
    double ra, dec;
    if (!raVal || !float_param(raVal, &ra) || !decVal || !float_param(decVal, &dec))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected ra and dec params");
        return;
    }

    const json_value *asyncVal = p.param("async");
    bool async = false;
    if (!asyncVal || !bool_param(asyncVal, &async))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected async param");
        return;
    }

    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    if (async && pPointingSource->CanSlewAsync())
    {
        if (!pPointingSource->SlewToCoordinatesAsync(ra, dec))
            response << jrpc_result(0);
        else
            response << jrpc_error(1, "failed to slew to coordinates asynchronously");
    }
    else if (!async && pPointingSource->CanSlew())
    {
        if (!pPointingSource->SlewToCoordinates(ra, dec))
            response << jrpc_result(0);
        else
            response << jrpc_error(1, "failed to slew to coordinates");
    }
    else
    {
        response << jrpc_error(1, "mount does not support slewing to coordinates");
    }
}

static void sync_coordinates(JObj& response, const json_value* params)
{
    Params p("ra", "dec", params);
    const json_value *raVal = p.param("ra");
    const json_value *decVal = p.param("dec");
    double ra, dec;
    if (!raVal || !float_param(raVal, &ra) || !decVal || !float_param(decVal, &dec))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected ra and dec params");
        return;
    }

    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    if (!pPointingSource->CanSync())
    {
        response << jrpc_error(1, "mount does not support sync to coordinates");
        return;
    }

    wxString errMsg;
    if (!pPointingSource->SyncToCoordinates(ra, dec, &errMsg))
        response << jrpc_result(0);
    else
        response << jrpc_error(1, errMsg.empty() ? "failed to sync to coordinates" : errMsg);
}

static wxMutex s_moveAxisLock;
static unsigned int s_moveAxisGeneration[2] = { 0, 0 };
static unsigned int s_moveAxisActive[2] = { 0, 0 };
static bool s_moveAxisCancelRequested[2] = { false, false };
static Scope *s_moveAxisScope[2] = { nullptr, nullptr };

static unsigned int register_move_axis_worker(Scope *scope, GuideAxis axis)
{
    wxMutexLocker lock(s_moveAxisLock);
    unsigned int generation = ++s_moveAxisGeneration[axis];
    ++s_moveAxisActive[axis];
    s_moveAxisCancelRequested[axis] = false;
    s_moveAxisScope[axis] = scope;
    return generation;
}

static void unregister_move_axis_worker(Scope *scope, GuideAxis axis, unsigned int generation)
{
    wxMutexLocker lock(s_moveAxisLock);
    if (s_moveAxisActive[axis] > 0)
        --s_moveAxisActive[axis];
    if (s_moveAxisActive[axis] == 0 && s_moveAxisScope[axis] == scope)
    {
        s_moveAxisCancelRequested[axis] = false;
        s_moveAxisScope[axis] = nullptr;
    }
}

static bool should_start_move_axis(Scope *scope, GuideAxis axis, unsigned int generation)
{
    wxMutexLocker lock(s_moveAxisLock);
    return s_moveAxisScope[axis] == scope && s_moveAxisGeneration[axis] == generation &&
           !s_moveAxisCancelRequested[axis];
}

static bool should_continue_move_axis(Scope *scope, GuideAxis axis, unsigned int generation)
{
    wxMutexLocker lock(s_moveAxisLock);
    return s_moveAxisScope[axis] == scope && s_moveAxisGeneration[axis] == generation &&
           !s_moveAxisCancelRequested[axis];
}

static bool should_stop_move_axis(Scope *scope, GuideAxis axis, unsigned int generation)
{
    wxMutexLocker lock(s_moveAxisLock);
    return s_moveAxisScope[axis] == scope && s_moveAxisGeneration[axis] == generation;
}

static bool request_move_axis_stop(Scope *scope, GuideAxis axis)
{
    wxMutexLocker lock(s_moveAxisLock);
    if (s_moveAxisScope[axis] == scope && s_moveAxisActive[axis] > 0)
    {
        s_moveAxisCancelRequested[axis] = true;
        return true;
    }
    return false;
}

void EventServer::CancelMoveAxisWorkers(Scope *scope)
{
    if (!scope)
        return;

    {
        wxMutexLocker lock(s_moveAxisLock);
        for (int axis = GUIDE_RA; axis <= GUIDE_DEC; ++axis)
        {
            if (s_moveAxisScope[axis] == scope && s_moveAxisActive[axis] > 0)
                s_moveAxisCancelRequested[axis] = true;
        }
    }

    bool loggedWait = false;
    wxStopWatch swatch;
    while (true)
    {
        bool active = false;
        {
            wxMutexLocker lock(s_moveAxisLock);
            for (int axis = GUIDE_RA; axis <= GUIDE_DEC; ++axis)
            {
                if (s_moveAxisScope[axis] == scope && s_moveAxisActive[axis] > 0)
                {
                    active = true;
                    break;
                }
            }
        }

        if (!active)
            return;

        if (!loggedWait && swatch.Time() >= 5000)
        {
            Debug.Write("EventServer::CancelMoveAxisWorkers still waiting for move-axis workers\n");
            loggedWait = true;
        }

        wxMilliSleep(10);
    }
}

class ASCOMMoveAxisThread : public wxThread
{
    Scope *m_scope;
    GuideAxis m_axis;
    double m_rate;
    int m_durationMs;
    unsigned int m_generation;

public:
    ASCOMMoveAxisThread(Scope *scope, GuideAxis axis, double rate, int durationMs, unsigned int generation)
        : wxThread(wxTHREAD_DETACHED), m_scope(scope), m_axis(axis), m_rate(rate), m_durationMs(durationMs),
          m_generation(generation)
    {
    }

    wxThread::ExitCode Entry() override
    {
#if defined(__WINDOWS__)
        HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
        Debug.Write(wxString::Format("ASCOMMoveAxisThread CoInitializeEx returns %x\n", hr));
#endif

        bool sendStop = false;
        bool startMove = m_rate == 0.0 ? should_stop_move_axis(m_scope, m_axis, m_generation) :
                                         should_start_move_axis(m_scope, m_axis, m_generation);
        if (startMove && m_scope && m_scope->IsConnected())
        {
            wxString errMsg;
            bool moveError = m_scope->ASCOM_MoveAxis(m_axis, m_rate, &errMsg);
            if (moveError)
                Debug.Write(wxString::Format("ASCOMMoveAxisThread: MoveAxis(%d, %.9g) failed: %s\n", (int) m_axis,
                                             m_rate, errMsg));

            if (!moveError && m_rate != 0.0)
            {
                int elapsed = 0;
                while (elapsed < m_durationMs && should_continue_move_axis(m_scope, m_axis, m_generation))
                {
                    int sleepMs = wxMin(50, m_durationMs - elapsed);
                    wxMilliSleep(sleepMs);
                    elapsed += sleepMs;
                }

                sendStop = should_stop_move_axis(m_scope, m_axis, m_generation);
            }
        }
        else if (m_rate != 0.0)
        {
            sendStop = should_stop_move_axis(m_scope, m_axis, m_generation);
        }

        if (sendStop && m_scope && m_scope->IsConnected())
        {
            wxString errMsg;
            if (m_scope->ASCOM_MoveAxis(m_axis, 0.0, &errMsg))
                Debug.Write(wxString::Format("ASCOMMoveAxisThread: MoveAxis(%d, 0) failed: %s\n", (int) m_axis,
                                             errMsg));
        }

#if defined(__WINDOWS__)
        if (SUCCEEDED(hr))
            CoUninitialize();
#endif

        unregister_move_axis_worker(m_scope, m_axis, m_generation);

        return (wxThread::ExitCode) 0;
    }
};

static bool start_ascom_move_axis_thread(Scope *scope, GuideAxis axis, double rate, int durationMs, unsigned int generation,
                                         wxString *error)
{
    ASCOMMoveAxisThread *thread = new ASCOMMoveAxisThread(scope, axis, rate, durationMs, generation);
    if (thread->Create() != wxTHREAD_NO_ERROR)
    {
        unregister_move_axis_worker(scope, axis, generation);
        delete thread;
        *error = "failed to create move axis thread";
        return false;
    }
    thread->SetPriority(WXTHREAD_MAX_PRIORITY);
    if (thread->Run() != wxTHREAD_NO_ERROR)
    {
        unregister_move_axis_worker(scope, axis, generation);
        delete thread;
        *error = "failed to start move axis thread";
        return false;
    }
    return true;
}

static void move_axis(JObj& response, const json_value *params)
{
    Params p("axis", "rate", "duration", params);
    GuideAxis a;

    const json_value *axisVal = p.param("axis");
    if (!axisVal || !(axisVal->type == JSON_INT) || axisVal->int_value < 0 || axisVal->int_value > 1)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS,
                               "invalid axis: expected 0 (RA/primary) or 1 (Dec/secondary)");
        return;
    }
    a = (GuideAxis) axisVal->int_value;

    const json_value *rateVal = p.param("rate");
    double rate;
    if (!rateVal || !float_param(rateVal, &rate))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected numeric rate param");
        return;
    }
    if (!std::isfinite(rate))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "invalid rate: expected finite numeric value");
        return;
    }

    int durationMs = 0;
    const json_value *durationVal = p.param("duration");
    if (rate != 0.0)
    {
        if (!durationVal || durationVal->type != JSON_INT || durationVal->int_value < 0)
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected non-negative duration param");
            return;
        }
        durationMs = durationVal->int_value;
    }

    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }
    if (pPointingSource->CanMoveAxis(a))
    {
        if (rate == 0.0 && request_move_axis_stop(pPointingSource, a))
        {
            response << jrpc_result(0);
            return;
        }

        if (rate != 0.0)
        {
            std::vector<Scope::AxisRate> axisRates;
            if (pPointingSource->GetAxisRates(a, &axisRates))
            {
                response << jrpc_error(1, "failed to get axis rates for MoveAxis rate validation");
                return;
            }

            double remappedRate = rate;
            if (!axis_rate_supported(axisRates, rate, &remappedRate))
            {
                response << jrpc_error(
                    JSONRPC_INVALID_PARAMS,
                    wxString::Format("invalid rate %.9g for axis %d; supported absolute rate ranges are %s", rate,
                                     (int) a, axis_rate_ranges_string(axisRates)));
                return;
            }
            if (remappedRate != rate)
            {
                Debug.Write(wxString::Format("EventServer::move_axis remapped axis %d rate %.9g to %.9g\n", (int) a,
                                             rate, remappedRate));
                rate = remappedRate;
            }
        }

        unsigned int generation = register_move_axis_worker(pPointingSource, a);
        wxString error;
        if (start_ascom_move_axis_thread(pPointingSource, a, rate, durationMs, generation, &error))
            response << jrpc_result(0);
        else
            response << jrpc_error(1, error);
    }
    else
    {
        response << jrpc_error(1, "mount does not support moving axes at variable rates");
    }
}

static void get_axis_rates(JObj& response, const json_value *params)
{
    Params p("axis", params);
    GuideAxis a;

    const json_value *axisVal = p.param("axis");
    if (!axisVal || !(axisVal->type == JSON_INT) || axisVal->int_value < 0 || axisVal->int_value > 1)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS,
                               "invalid axis: expected 0 (RA/primary) or 1 (Dec/secondary)");
        return;
    }
    a = (GuideAxis) axisVal->int_value;

    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    std::vector<Scope::AxisRate> axisRates;
    if (!pPointingSource->GetAxisRates(a, &axisRates))
    {
        JAry ary = axis_rates_json(axisRates);
        response << jrpc_result(ary);
    }
    else
    {
        response << jrpc_error(1, "failed to get axis rates");
    }
}

static void abort_slew(JObj& response, const json_value *params)
{
    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }
    if (!pPointingSource->AbortSlew())
        response << jrpc_result(0);
    else
        response << jrpc_error(1, "failed to abort slew");
}

static void poll_mount_slewing(JObj& response, const json_value *params)
{
    if (pPointingSource && pPointingSource->IsConnected())
    {
        bool slewing = pPointingSource->Slewing();
        response << jrpc_result(slewing);
    }
    else
    {
        response << jrpc_error(1, "mount not connected");
    }
}

static void set_surf_mode(JObj& response, const json_value *params)
{
    Params p("mode", params);

    const json_value *val = p.param("mode");
    if (!val || val->type != JSON_BOOL)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected mode param");
        return;
    }
    bool mode = val->int_value;
    if (pFrame->pGuider)
        pFrame->pGuider->m_SolarSystemObject.Set_SurfaceDetectionMode(mode);
    response << jrpc_result(0);
}

static void get_surf_mode(JObj& response, const json_value *params)
{
    if (pFrame->pGuider)
        response << NV("surf", pFrame->pGuider->m_SolarSystemObject.GetSurfaceTrackingState());
    else
        response << jrpc_error(1, "guider not connected");
}

static void get_process_id(JObj& response, const json_value *params)
{
    // Return the process ID of the server
    response << jrpc_result((int) wxGetProcessId());
}

// The single event-server client that enabled shared-memory frame export.
// Export is auto-disabled if this client disconnects.
static wxSocketClient *s_frameExportClient = nullptr;

static void set_frame_export(JObj& response, const json_value *params, ClientData *cd)
{
    Params p("enabled", params);
    const json_value *val = p.param("enabled");
    if (!val || val->type != JSON_BOOL)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected enabled param");
        return;
    }
    bool enable = val->int_value != 0;

    // Single owner: reject if another client currently holds frame export.
    if (s_frameExportClient && cd && s_frameExportClient != cd->cli)
    {
        response << jrpc_error(1, "frame export already in use by another client");
        return;
    }

    if (enable && !FrameExport::Available())
    {
        response << jrpc_error(1, "frame export not supported on this platform");
        return;
    }

    if (!FrameExport::Enable(enable))
    {
        s_frameExportClient = nullptr;
        response << jrpc_error(1, "failed to open shared memory frame channel");
        return;
    }

    s_frameExportClient = (enable && FrameExport::IsEnabled() && cd) ? cd->cli : nullptr;
    response << jrpc_result(FrameExport::IsEnabled() ? 1 : 0);
}

static void get_frame_export(JObj& response, const json_value *params)
{
    response << jrpc_result(FrameExport::IsEnabled() ? 1 : 0);
}

static void get_planet_state(JObj& response, const json_value *params)
{
    if (!pFrame->pGuider)
    {
        response << jrpc_error(1, "guider not connected");
        return;
    }

    SolarSystemObject::PlanetHandoverState s;
    pFrame->pGuider->m_SolarSystemObject.GetPlanetHandoverState(s);

    Debug.Write(wxString::Format("evsrv: get_planet_state -> mode=%d detected=%d canny_high=%d "
                                 "radius=%d min=%d max=%d\n",
                                 s.planetaryMode, s.detected, s.cannyHigh, s.radius, s.minRadius, s.maxRadius));

    JObj rslt;
    rslt << NV("planetary_mode", s.planetaryMode) << NV("detected", s.detected) << NV("cx", s.cx) << NV("cy", s.cy)
         << NV("radius", s.radius) << NV("min_radius", s.minRadius) << NV("max_radius", s.maxRadius)
         << NV("canny_high", s.cannyHigh);
    response << jrpc_result(rslt);
}

static void set_planet_thresholds(JObj& response, const json_value *params)
{
    if (!params || params->type != JSON_OBJECT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected thresholds object");
        return;
    }
    if (!pFrame->pGuider)
    {
        response << jrpc_error(1, "guider not connected");
        return;
    }

    Params p("high", "low", params);
    const json_value *hv = p.param("high");
    if (!hv || hv->type != JSON_INT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected high threshold");
        return;
    }
    int high = hv->int_value;
    const json_value *lv = p.param("low");
    int low = (lv && lv->type == JSON_INT) ? lv->int_value : wxMax(high / 2, 1);

    Debug.Write(wxString::Format("evsrv: set_planet_thresholds <- high=%d low=%d\n", high, low));

    pFrame->pGuider->m_SolarSystemObject.SetDetectionThresholds(high, low);
    response << jrpc_result(0);
}

static void set_planet_detection(JObj& response, const json_value *params)
{
    if (!params || params->type != JSON_OBJECT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected detection result object");
        return;
    }
    if (!pFrame->pGuider)
    {
        response << jrpc_error(1, "guider not connected");
        return;
    }

    Params p("frame", params);
    auto getInt = [&p](const char *name, int def) -> int {
        const json_value *v = p.param(name);
        if (!v)
            return def;
        if (v->type == JSON_INT || v->type == JSON_BOOL)
            return v->int_value;
        if (v->type == JSON_FLOAT)
            return (int) v->float_value;
        return def;
    };
    auto getDouble = [&p](const char *name, double def) -> double {
        const json_value *v = p.param(name);
        if (!v)
            return def;
        if (v->type == JSON_FLOAT)
            return v->float_value;
        if (v->type == JSON_INT)
            return v->int_value;
        return def;
    };
    auto getBool = [&p](const char *name, bool def) -> bool {
        const json_value *v = p.param(name);
        if (!v)
            return def;
        if (v->type == JSON_BOOL || v->type == JSON_INT)
            return v->int_value != 0;
        return def;
    };

    SolarSystemObject::RemoteDetection r;
    r.frame = (uint32_t) getInt("frame", 0);
    r.detected = getBool("detected", false);
    r.x = (float) getDouble("x", 0);
    r.y = (float) getDouble("y", 0);
    r.radius = getInt("radius", 0);
    r.minRadius = getInt("min_radius", 0);
    r.maxRadius = getInt("max_radius", 0);
    r.peak = getInt("peak", 0);
    r.features = getInt("features", 0);
    r.mass = getDouble("mass", 0);
    r.snr = getDouble("snr", 0);
    r.sharpness = getDouble("sharpness", 0);
    r.quality = getDouble("quality", 0);
    r.dispersion = getDouble("dispersion", 0);
    r.roiX = getInt("roi_x", 0);
    r.roiY = getInt("roi_y", 0);
    r.roiW = getInt("roi_w", 0);
    r.roiH = getInt("roi_h", 0);

    pFrame->pGuider->m_SolarSystemObject.SetRemoteDetectionResult(r);
    response << jrpc_result(0);
}

static void set_planet_size(JObj& response, const json_value *params)
{
    Params p("radii", params);
    const json_value *val = p.param("MinRadius");
    if (!val || val->type != JSON_INT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected min radius");
        return;
    }
    int minRadius = val->int_value;

    val = p.param("MaxRadius");
    if (!val || val->type != JSON_INT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected max radius");
        return;
    }
    int maxRadius = val->int_value;

    if (pFrame->pGuider)
    {
        if (!pFrame->pGuider->m_SolarSystemObject.SetLimitsPersisted(minRadius, maxRadius))
        {
            response << jrpc_error(1, "invalid data");
            return;
        }
    }
    response << jrpc_result(0);
}

static void get_mount_coords(JObj& response, const json_value *params, ClientData *cd)
{
    EXT_PROTOCOL_VERSION protocol = cd ? cd->extProtocol : EXT_PROTOCOL_SOLAR1;
    if (!protocol_is_solar2(protocol))
    {
        JObj rslt;
        double ra, dec, st;
        if (pPointingSource && pPointingSource->IsConnected() && !pPointingSource->GetCoordinates(&ra, &dec, &st))
        {
            rslt << NV("ra", ra) << NV("dec", dec) << NV("sidereal", st);
            response << jrpc_result(rslt);
        }
        else
        {
            response << jrpc_error(1, "mount not connected");
        }
        return;
    }

    Params p("pier_side", params);
    bool includePierSide = false;
    if (const json_value *val = p.param("pier_side"))
    {
        if (!bool_param(val, &includePierSide))
        {
            response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected boolean pier_side");
            return;
        }
    }

    JObj rslt;
    double ra, dec, st;
    bool haveCoords = false;
    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    if (!pPointingSource->GetCoordinates(&ra, &dec, &st))
    {
        rslt << NV("ra", ra) << NV("dec", dec) << NV("sidereal", st);
        wxString ha = hour_angle_log_value(ra, st);
        if (ha != "N/A")
            rslt << NV("ha", hour_angle(ra, st));
        if (includePierSide)
        {
            PierSide pierSide = pPointingSource->SideOfPier();
            wxString pierSideLog = pier_side_name(pierSide);
            rslt << NV("pier_side", pierSideLog);
            Debug.Write(wxString::Format("get_mount_coords ra=%.6f dec=%.6f sidereal=%.6f ha=%s pier_side=%s\n", ra, dec, st,
                                         ha, pierSideLog));
        }
        else
        {
            Debug.Write(wxString::Format("get_mount_coords ra=%.6f dec=%.6f sidereal=%.6f ha=%s\n", ra, dec, st, ha));
        }

        haveCoords = true;
    }

    double alt, az;
    if (!pPointingSource->GetMountAltAz(&alt, &az))
    {
        rslt << NV("alt", alt) << NV("az", az);
        haveCoords = true;
    }
    if (haveCoords)
        response << jrpc_result(rslt);
    else
        response << jrpc_error(1, "failed to get mount coordinates");
}

static void get_mount_side_of_pier(JObj& response, const json_value *params)
{
    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    PierSide side = pPointingSource->SideOfPier();
    if (side == PIER_SIDE_UNKNOWN)
    {
        response << jrpc_error(1, "mount does not support side of pier");
        return;
    }

    JObj rslt;
    rslt << NV("pier_side", pier_side_name(side));
    response << jrpc_result(rslt);
}

static void get_mount_destination_side_of_pier(JObj& response, const json_value *params)
{
    Params p("ra", "dec", params);
    const json_value *raVal = p.param("ra");
    const json_value *decVal = p.param("dec");
    double ra, dec;
    if (!raVal || !float_param(raVal, &ra) || !decVal || !float_param(decVal, &dec))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected ra and dec params");
        return;
    }

    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    PierSide side = pPointingSource->DestinationSideOfPier(ra, dec);
    if (side == PIER_SIDE_UNKNOWN)
    {
        response << jrpc_error(1, "mount does not support destination side of pier");
        return;
    }

    JObj rslt;
    rslt << NV("pier_side", pier_side_name(side));
    response << jrpc_result(rslt);
}

static void get_site_coords(JObj& response, const json_value *params)
{
    JObj rslt;
    double latitude, longitude, elevation;

    if (pPointingSource && pPointingSource->IsConnected())
    {
        if (!pPointingSource->GetSiteLatLong(&latitude, &longitude))
        {
            rslt << NV("latitude", latitude) << NV("longitude", longitude);
            if (!pPointingSource->GetSiteElevation(&elevation))
                rslt << NV("elevation", elevation);
            response << jrpc_result(rslt);
        }
    }
    else
    {
        response << jrpc_error(1, "mount not connected");
    }
}

// {"method": "set_site_coords", "params": {"latitude":47.5, "longitude":-122.3, "elevation":50.0}, "id":1}
// All three fields are optional individually, but latitude and longitude must be supplied as a
// pair (the underlying ASCOM SiteLatitude/SiteLongitude pair describes one location). Pass just
// elevation to update only that. Returns an error if the driver doesn't expose the property as
// writable or if any update fails. Successful partial updates are NOT rolled back.
static void set_site_coords(JObj& response, const json_value *params)
{
    Params p("latitude", "longitude", "elevation", params);
    const json_value *pLat = p.param("latitude");
    const json_value *pLon = p.param("longitude");
    const json_value *pElev = p.param("elevation");

    if (!pLat && !pLon && !pElev)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS,
                               "expected at least one of latitude, longitude, elevation");
        return;
    }

    // Latitude and longitude must be supplied together.
    if ((pLat == nullptr) != (pLon == nullptr))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS,
                               "latitude and longitude must be supplied together");
        return;
    }

    double latitude = 0, longitude = 0, elevation = 0;
    if (pLat && (!float_param(pLat, &latitude) || !float_param(pLon, &longitude)))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "latitude/longitude must be numeric");
        return;
    }
    if (pElev && !float_param(pElev, &elevation))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "elevation must be numeric");
        return;
    }

    if (pLat && (latitude < -90.0 || latitude > 90.0))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "latitude out of range (-90..90)");
        return;
    }
    if (pLon && (longitude < -180.0 || longitude > 180.0))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "longitude out of range (-180..180)");
        return;
    }

    if (!pPointingSource || !pPointingSource->IsConnected())
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    if (pLat && pPointingSource->SetSiteLatLong(latitude, longitude))
    {
        response << jrpc_error(1, "failed to set site latitude/longitude (driver may not support it)");
        return;
    }

    if (pElev && pPointingSource->SetSiteElevation(elevation))
    {
        response << jrpc_error(1, "failed to set site elevation (driver may not support it)");
        return;
    }

    response << jrpc_result(0);
}

static void get_mount_tracking(JObj& response, const json_value *params, ClientData *cd)
{
    if (!pPointingSource || !pPointingSource->IsConnected() || !pFrame->pGuider)
    {
        response << jrpc_error(1, "mount not connected");
        return;
    }

    bool trackingValid = false;
    bool tracking = false;
    wxString rate = "Sidereal";
    bool rateValid = false;

    // Structured fields for newer clients
    bool offsetsValid = false;
    double raOffset = 0.0;
    double decOffset = 0.0;

    rateValid = pFrame->pGuider->m_SolarSystemObject.GetMountTrackingState(trackingValid, tracking, rate,
        offsetsValid, raOffset, decOffset);

    if (!trackingValid && !rateValid)
    {
        response << jrpc_error(1, "mount tracking not available");
        return;
    }

    JObj rslt;

    if (trackingValid)
        rslt << NV("tracking", tracking);

    if (rateValid)
        rslt << NV("rate", rate);

    EXT_PROTOCOL_VERSION protocol = cd ? cd->extProtocol : EXT_PROTOCOL_SOLAR1;
    if (offsetsValid && protocol_is_solar2(protocol))
    {
        rslt << NV("ra_offset", raOffset);
        rslt << NV("dec_offset", decOffset);
    }

    response << jrpc_result(rslt);
}

static void set_mount_tracking(JObj& response, const json_value *params)
{
    Params p("params", params);
    const json_value *val = p.param("rate");
    if (!val || val->type != JSON_STRING)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected rate param");
        return;
    }
    wxString rate = val->string_value;

    val = p.param("ra_rate");
    double ra_rate = 0;
    if (val && val->type == JSON_FLOAT)
        ra_rate = val->float_value;
    val = p.param("dec_rate");
    double dec_rate = 0;
    if (val && val->type == JSON_FLOAT)
        dec_rate = val->float_value;

    if (pFrame->pGuider && pFrame->pGuider->m_SolarSystemObject.SetMountTrackingRate(rate, ra_rate, dec_rate))
        response << jrpc_result(0);
    else
        response << jrpc_error(1, "failed to set mount tracking rate");
}

static void get_cal_data(JObj& response, const json_value *params)
{
    JObj rslt;

    Mount *mount = (!pSecondaryMount || AO) ? pMount : pSecondaryMount;
    if (mount && mount->IsCalibrated())
    {
        CalibrationDetails d;
        mount->LoadCalibrationDetails(&d);

        JObj details;
        details << NV("focalLength", d.focalLength);
        details << NV("imageScale", d.imageScale);
        details << NV("raGuideSpeed", d.raGuideSpeed);
        details << NV("decGuideSpeed", d.decGuideSpeed);
        details << NV("orthoError", d.orthoError);
        details << NV("origBinning", d.origBinning);
        details << NV("raStepCount", d.raStepCount);
        details << NV("decStepCount", d.decStepCount);
        details << NV("origTimestamp", d.origTimestamp);
        if (d.origPierSide == PIER_SIDE_UNKNOWN)
            details << NV("origPierSide", "?");
        else
            details << NV("origPierSide", d.origPierSide == PIER_SIDE_EAST ? "east" : "west");

        JAry raSteps;
        for (unsigned int i = 0; i < d.raSteps.size(); i++)
        {
            PHD_Point pt(d.raSteps[i].x, d.raSteps[i].y);
            JObj t;
            t << pt;
            raSteps << t;
        }
        details << NV("raSteps", raSteps);

        JAry decSteps;
        for (unsigned int i = 0; i < d.decSteps.size(); i++)
        {
            PHD_Point pt(d.decSteps[i].x, d.decSteps[i].y);
            JObj t;
            t << pt;
            decSteps << t;
        }
        details << NV("decSteps", decSteps);

        Calibration b;
        mount->GetLastCalibration(&b);
        JObj last;
        last << NV("xRate", b.xRate);
        last << NV("yRate", b.yRate);
        last << NV("xAngle", b.xAngle);
        last << NV("yAngle", b.yAngle);
        last << NV("declination", b.declination);
        last << NV("rotatorAngle", b.rotatorAngle);
        last << NV("binning", b.binning);
        if (b.pierSide == PIER_SIDE_UNKNOWN)
            last << NV("pierSide", "?");
        else
            last << NV("pierSide", b.pierSide == PIER_SIDE_EAST ? "east" : "west");
        last << NV("timestamp", b.timestamp);
        last << NV("isValid", b.isValid);

        rslt << NV("details", details);
        rslt << NV("last", last);
        response << jrpc_result(rslt);
    }
    else
    {
        response << jrpc_error(1, "mount not calibrated");
    }
}

static void get_drift_measurement(JObj& response, const json_value *params)
{
    GuidingAssistant::DriftMeasurement measurement;
    JObj rslt;
    if (!GuidingAssistant::GetLastDriftMeasurement(&measurement))
    {
        rslt << NV("available", false);
        response << jrpc_result(rslt);
        return;
    }

    JObj cameraPxPerMin;
    cameraPxPerMin << NV("x", measurement.cameraRatePxPerMin.X) << NV("y", measurement.cameraRatePxPerMin.Y);

    JObj mountPxPerMin;
    mountPxPerMin << NV("ra", measurement.mountRatePxPerMin.X) << NV("dec", measurement.mountRatePxPerMin.Y);

    JObj cameraArcsecPerMin;
    cameraArcsecPerMin << NV("x", measurement.cameraRateArcsecPerMin.X) << NV("y", measurement.cameraRateArcsecPerMin.Y);

    JObj mountArcsecPerMin;
    mountArcsecPerMin << NV("ra", measurement.mountRateArcsecPerMin.X) << NV("dec", measurement.mountRateArcsecPerMin.Y);

    const Calibration& cal = measurement.calibration;
    JObj calibration;
    calibration << NV("source", measurement.calibrationSource);
    calibration << NV("used_step_guider", measurement.usedStepGuider);
    calibration << NV("is_valid", cal.isValid);
    if (cal.isValid)
    {
        calibration << NV("x_angle", cal.xAngle);
        calibration << NV("y_angle", cal.yAngle);
        calibration << NV("x_angle_degrees", degrees(cal.xAngle));
        calibration << NV("y_angle_degrees", degrees(cal.yAngle));
        calibration << NV("orthogonality_error_degrees", degrees(fabs(fabs(norm_angle(cal.xAngle - cal.yAngle)) - M_PI / 2.)));
        calibration << NV("x_rate", cal.xRate);
        calibration << NV("y_rate", cal.yRate);
        calibration << NV("x_rate_px_per_sec", cal.xRate * 1000.0);
        calibration << NV("y_rate_px_per_sec", cal.yRate * 1000.0);
        if (cal.declination == UNKNOWN_DECLINATION)
        {
            calibration << NV("declination", NULL_VALUE);
            calibration << NV("declination_degrees", NULL_VALUE);
        }
        else
        {
            calibration << NV("declination", cal.declination);
            calibration << NV("declination_degrees", degrees(cal.declination));
        }
        calibration << NV("rotator_angle", cal.rotatorAngle);
        calibration << NV("binning", cal.binning);
        if (cal.pierSide == PIER_SIDE_UNKNOWN)
            calibration << NV("pier_side", "?");
        else
            calibration << NV("pier_side", cal.pierSide == PIER_SIDE_EAST ? "east" : "west");
        calibration << NV("ra_guide_parity", cal.raGuideParity == GUIDE_PARITY_EVEN ? "+" :
                                                 cal.raGuideParity == GUIDE_PARITY_ODD ? "-" : "?");
        calibration << NV("dec_guide_parity", cal.decGuideParity == GUIDE_PARITY_EVEN ? "+" :
                                                  cal.decGuideParity == GUIDE_PARITY_ODD ? "-" : "?");
        calibration << NV("timestamp", cal.timestamp);
    }

    rslt << NV("available", true);
    rslt << NV("start_time", measurement.startTime);
    rslt << NV("end_time", measurement.endTime);
    rslt << NV("duration", measurement.duration);
    rslt << NV("samples", measurement.sampleCount);
    rslt << NV("pixel_scale", measurement.pixelScale);
    rslt << NV("camera_rate_px_per_min", cameraPxPerMin);
    rslt << NV("mount_rate_px_per_min", mountPxPerMin);
    rslt << NV("camera_rate_arcsec_per_min", cameraArcsecPerMin);
    rslt << NV("mount_rate_arcsec_per_min", mountArcsecPerMin);
    rslt << NV("calibration", calibration);

    response << jrpc_result(rslt);
}

// Use with caution: this will clear the calibration
static void set_cal_step(JObj& response, const json_value *params)
{
    Params p("step", params);

    const json_value *val = p.param("step");
    if (!val || val->type != JSON_INT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected step param");
        return;
    }
    int step = val->int_value;

    val = p.param("dist");
    if (!val || val->type != JSON_INT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected dist param");
        return;
    }
    int dist = val->int_value;

    if (pPointingSource &&
        ((pPointingSource->GetCalibrationDistance() != dist) || (pPointingSource->GetCalibrationDuration() != step)))
    {
        pPointingSource->SetCalibrationDistance(dist);
        pPointingSource->SetCalibrationDuration(step);
        if (pMount)
        {
            pMount->ClearCalibration();
            if (pMount->IsStepGuider() && pSecondaryMount)
                pSecondaryMount->ClearCalibration();

            double defMinMove =
                GuideAlgorithm::SmartDefaultMinMove(pFrame->GetFocalLength(), pCamera->GetCameraPixelSize(), pCamera->GetBinning());
            pMount->GetXGuideAlgorithm()->SetMinMove(defMinMove);
            pMount->GetYGuideAlgorithm()->SetMinMove(defMinMove);
        }
    }
    response << jrpc_result(0);
}

static void set_iflink(JObj& response, const json_value *params)
{
    Params p("port", params);
    const json_value *val = p.param("port");
    if (!val || val->type != JSON_INT)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected port number");
        return;
    }
    if (pCamera)
        pCamera->SetProperty("port", val->int_value);
    response << jrpc_result(0);
}

static void set_iflink_cam(JObj& response, const json_value *params)
{
    Params p("name", params);
    const json_value *val = p.param("name");
    if (!val || val->type != JSON_STRING)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected name");
        return;
    }
    if (pCamera)
        pCamera->SetProperty("name", val->string_value);
    response << jrpc_result(0);
}

// set_variable_delay values are in units of seconds to match the UI convention in the Advanced Settings dialog
static void set_variable_delay_settings(JObj& response, const json_value *params)
{
    Params p("Enabled", "ShortDelaySeconds", "LongDelaySeconds", params);
    const json_value *p0 = p.param("Enabled");
    const json_value *p1 = p.param("ShortDelaySeconds");
    const json_value *p2 = p.param("LongDelaySeconds");
    bool enabled;
    double shortDelaySec;
    double longDelaySec;
    if (!p0 || !p1 || !p2 || !bool_param(p0, &enabled) || !float_param(p1, &shortDelaySec) || !float_param(p2, &longDelaySec))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected Enabled, ShortDelaySeconds, LongDelaySeconds params)");
        return;
    }
    VarDelayCfg currParams;
    pFrame->SetVariableDelayConfig(enabled, (int) shortDelaySec * 1000, (int) longDelaySec * 1000);
    response << jrpc_result(0);
}

static void get_limit_frame(JObj& response, const json_value *params)
{
    JObj rslt;

    if (!pCamera || !pCamera->HasFrameLimiting || pCamera->LimitFrame.IsEmpty())
        rslt << NV("roi", NULL_VALUE);
    else
        rslt << NV("roi", pCamera->LimitFrame);
    response << jrpc_result(rslt);
}

static void set_limit_frame(JObj& response, const json_value *params)
{
    Params p("roi", params);
    const json_value *j = p.param("roi");
    if (!j)
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "missing required param `roi`");
        return;
    }
    wxRect roi;
    if (j->type != JSON_NULL && !parse_rect(&roi, j))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "invalid ROI param");
        return;
    }
    if (!pCamera)
    {
        response << jrpc_error(1, "no guide camera");
        return;
    }
    if (!pCamera->HasFrameLimiting)
    {
        response << jrpc_error(1, "guide camera does not support frame limiting");
        return;
    }

    VERIFY_GUIDER(response);

    wxString errorMessage;
    bool err = pCamera->SetLimitFrame(roi, 1, &errorMessage);

    if (err)
        response << jrpc_error(1, errorMessage);
    else
        response << jrpc_result(0);
}

static GUIDE_DIRECTION dir_param(const json_value *p)
{
    if (!p || p->type != JSON_STRING)
        return GUIDE_DIRECTION::NONE;

    struct
    {
        const char *s;
        GUIDE_DIRECTION d;
    } dirs[] = {
        { "n", GUIDE_DIRECTION::NORTH },   { "s", GUIDE_DIRECTION::SOUTH },     { "e", GUIDE_DIRECTION::EAST },
        { "w", GUIDE_DIRECTION::WEST },    { "north", GUIDE_DIRECTION::NORTH }, { "south", GUIDE_DIRECTION::SOUTH },
        { "east", GUIDE_DIRECTION::EAST }, { "west", GUIDE_DIRECTION::WEST },   { "up", GUIDE_DIRECTION::UP },
        { "down", GUIDE_DIRECTION::DOWN }, { "left", GUIDE_DIRECTION::LEFT },   { "right", GUIDE_DIRECTION::RIGHT },
    };

    for (unsigned int i = 0; i < WXSIZEOF(dirs); i++)
        if (wxStricmp(p->string_value, dirs[i].s) == 0)
            return dirs[i].d;

    return GUIDE_DIRECTION::NONE;
}

static GUIDE_DIRECTION opposite(GUIDE_DIRECTION d)
{
    switch (d)
    {
    case UP:
        return DOWN;
    case DOWN:
        return UP;
    case LEFT:
        return RIGHT;
    case RIGHT:
        return LEFT;
    default:
        return d;
    }
}

static void guide_pulse(JObj& response, const json_value *params)
{
    Params p("amount", "direction", "which", params);

    const json_value *amount = p.param("amount");
    if (!amount || amount->type != JSON_INT)
    {
        response << jrpc_error(1, "expected amount param");
        return;
    }

    GUIDE_DIRECTION dir = dir_param(p.param("direction"));
    if (dir == GUIDE_DIRECTION::NONE)
    {
        response << jrpc_error(1, "expected direction param");
        return;
    }

    WHICH_MOUNT which = which_mount(p.param("which"));
    Mount *m = nullptr;
    switch (which)
    {
    case MOUNT:
        m = TheScope();
        break;
    case AO:
        m = TheAO();
        break;
    case WHICH_MOUNT_BOTH:
    case WHICH_MOUNT_ERR:
        response << jrpc_error(1, "invalid 'which' param");
        return;
    }

    if (!m || !m->IsConnected())
    {
        response << jrpc_error(1, "device not connected");
        return;
    }

    VERIFY_GUIDER(response);

    if (pFrame->pGuider->IsCalibratingOrGuiding() || m->IsBusy())
    {
        response << jrpc_error(1, "cannot issue guide pulse while calibrating or guiding");
        return;
    }

    int duration = amount->int_value;
    if (duration < 0)
    {
        duration = -duration;
        dir = opposite(dir);
    }

    pFrame->ScheduleManualMove(m, dir, duration);

    response << jrpc_result(0);
}

static const char *parity_str(GuideParity p)
{
    switch (p)
    {
    case GUIDE_PARITY_EVEN:
        return "+";
    case GUIDE_PARITY_ODD:
        return "-";
    default:
        return "?";
    }
}

static void get_calibration_data(JObj& response, const json_value *params)
{
    Params p("which", params);

    WHICH_MOUNT which = which_mount(p.param("which"));
    Mount *m = nullptr;
    switch (which)
    {
    case MOUNT:
        m = TheScope();
        break;
    case AO:
        m = TheAO();
        break;
    case WHICH_MOUNT_BOTH:
    case WHICH_MOUNT_ERR:
    {
        response << jrpc_error(1, "invalid 'which' param");
        return;
    }
    }

    if (!m || !m->IsConnected())
    {
        response << jrpc_error(1, "device not connected");
        return;
    }

    JObj rslt;
    rslt << NV("calibrated", m->IsCalibrated());

    if (m->IsCalibrated())
    {
        rslt << NV("xAngle", degrees(m->xAngle()), 1) << NV("xRate", m->xRate() * 1000.0, 3)
             << NV("xParity", parity_str(m->RAParity())) << NV("yAngle", degrees(m->yAngle()), 1)
             << NV("yRate", m->yRate() * 1000.0, 3) << NV("yParity", parity_str(m->DecParity()))
             << NV("declination", degrees(m->GetCalibrationDeclination()));
    }

    response << jrpc_result(rslt);
}

static void set_cooler_state(JObj& response, const json_value *params)
{
    Params p("enabled", params);
    const json_value *val = p.param("enabled");
    bool enable;
    if (!val || !bool_param(val, &enable))
    {
        response << jrpc_error(JSONRPC_INVALID_PARAMS, "expected enabled boolean param");
        return;
    }

    if (!pCamera || !pCamera->Connected)
    {
        response << jrpc_error(1, "camera not connected");
        return;
    }

    if (!pCamera->HasCooler)
    {
        response << jrpc_error(1, "camera lacks a cooler");
        return;
    }

    if (pCamera->SetCoolerOn(enable))
    {
        response << jrpc_error(1, "failed to set cooler state");
        return;
    }

    if (enable)
    {
        double setpt = pConfig->Profile.GetDouble("/camera/CoolerSetpt", 10.0);
        if (pCamera->SetCoolerSetpoint(setpt))
        {
            response << jrpc_error(1, "failed to set cooler setpoint");
            return;
        }
    }

    response << jrpc_result(0);
}

static void get_cooler_status(JObj& response, const json_value *params)
{
    if (!pCamera || !pCamera->Connected)
    {
        response << jrpc_error(1, "camera not connected");
        return;
    }

    bool on;
    double setpoint, power, temperature;

    bool err = pCamera->GetCoolerStatus(&on, &setpoint, &power, &temperature);
    if (err)
    {
        response << jrpc_error(1, "failed to get cooler status");
        return;
    }

    JObj rslt;

    rslt << NV("coolerOn", on) << NV("temperature", temperature, 1);

    if (on)
    {
        rslt << NV("setpoint", setpoint, 1) << NV("power", power, 1);
    }

    response << jrpc_result(rslt);
}

static void get_sensor_temperature(JObj& response, const json_value *params)
{
    if (!pCamera || !pCamera->Connected)
    {
        response << jrpc_error(1, "camera not connected");
        return;
    }

    double temperature;
    bool err = pCamera->GetSensorTemperature(&temperature);
    if (err)
    {
        response << jrpc_error(1, "failed to get sensor temperature");
        return;
    }

    JObj rslt;
    rslt << NV("temperature", temperature, 1);

    response << jrpc_result(rslt);
}

static void export_config_settings(JObj& response, const json_value *params)
{
    wxString filename(MyFrame::GetDefaultFileDir() + PATHSEPSTR + "phd2_settings.txt");
    bool err = pConfig->SaveAll(filename);

    if (err)
    {
        response << jrpc_error(1, "export settings failed");
        return;
    }

    JObj rslt;
    rslt << NV("filename", filename);

    response << jrpc_result(rslt);
}

struct JRpcCall
{
    wxSocketClient *cli;
    const json_value *req;
    const json_value *method;
    JRpcResponse response;

    JRpcCall(wxSocketClient *cli_, const json_value *req_) : cli(cli_), req(req_), method(nullptr) { }
};

static void dump_request(const JRpcCall& call)
{
    Debug.Write(wxString::Format("evsrv: cli %p request: %s\n", call.cli, json_format(call.req)));
}

static void dump_response(const JRpcCall& call)
{
    wxString s(const_cast<JRpcResponse&>(call.response).str());

    // trim output for huge responses

    // this is very hacky operating directly on the string, but it's not
    // worth bothering to parse and reformat the response
    if (call.method && strcmp(call.method->string_value, "get_star_image") == 0)
    {
        size_t p0, p1;
        if ((p0 = s.find("\"pixels\":\"")) != wxString::npos && (p1 = s.find('"', p0 + 10)) != wxString::npos)
            s.replace(p0 + 10, p1 - (p0 + 10), "...");
    }

    Debug.Write(wxString::Format("evsrv: cli %p response: %s\n", call.cli, s));
}

// RPC handlers run on the UI thread; do not enter nested wx event loops here.
static bool handle_request(JRpcCall& call)
{
    RpcEntryGuard rpcGuard;

    const json_value *params;
    const json_value *id;

    dump_request(call);

    parse_request(call.req, &call.method, &params, &id);

    if (!call.method)
    {
        call.response << jrpc_error(JSONRPC_INVALID_REQUEST, "invalid request - missing method") << jrpc_id(0);
        return true;
    }

    if (params && !(params->type == JSON_ARRAY || params->type == JSON_OBJECT))
    {
        call.response << jrpc_error(JSONRPC_INVALID_REQUEST, "invalid request - params must be an array or object")
                      << jrpc_id(0);
        return true;
    }

    ClientData *cd = client_data(call.cli);
    EXT_PROTOCOL_VERSION protocol = cd ? cd->extProtocol : EXT_PROTOCOL_SOLAR1;

    static struct
    {
        const char *name;
        void (*fn)(JObj& response, const json_value *params);
        EXT_PROTOCOL_VERSION minProtocol;
        void (*fnWithClient)(JObj& response, const json_value *params, ClientData *cd);
    } methods[] = {
        { "clear_calibration", &clear_calibration },
        { "deselect_star", &deselect_star },
        { "get_exposure", &get_exposure },
        { "set_exposure", &set_exposure },
        { "get_exposure_durations", &get_exposure_durations },
        { "get_profiles", &get_profiles },
        { "get_profile", &get_profile },
        { "set_profile", &set_profile },
        { "get_connected", &get_connected },
        { "set_connected", &set_connected },
        { "get_calibrated", &get_calibrated },
        { "get_paused", &get_paused },
        { "set_paused", &set_paused },
        { "get_lock_position", &get_lock_position },
        { "set_lock_position", &set_lock_position },
        { "loop", &loop },
        { "stop_capture", &stop_capture },
        { "guide", &guide },
        { "dither", &dither },
        { "find_star", &find_star },
        { "get_pixel_scale", &get_pixel_scale },
        { "get_app_state", &get_app_state },
        { "flip_calibration", &flip_calibration },
        { "get_lock_shift_enabled", &get_lock_shift_enabled },
        { "set_lock_shift_enabled", &set_lock_shift_enabled },
        { "get_lock_shift_params", &get_lock_shift_params },
        { "set_lock_shift_params", &set_lock_shift_params },
        { "save_image", &save_image },
        { "get_star_image", &get_star_image },
        { "get_use_subframes", &get_use_subframes },
        { "get_search_region", &get_search_region },
        { "shutdown", &shutdown },
        { "get_camera_binning", &get_camera_binning },
        { "get_camera_frame_size", &get_camera_frame_size },
        { "get_current_equipment", &get_current_equipment },
        { "get_guide_output_enabled", &get_guide_output_enabled },
        { "set_guide_output_enabled", &set_guide_output_enabled },
        { "get_algo_param_names", &get_algo_param_names },
        { "get_algo_param", &get_algo_param },
        { "set_algo_param", &set_algo_param },
        { "get_dec_guide_mode", &get_dec_guide_mode },
        { "set_dec_guide_mode", &set_dec_guide_mode },
        { "get_settling", &get_settling },
        { "guide_pulse", &guide_pulse },
        { "get_calibration_data", &get_calibration_data },
        { "capture_single_frame", &capture_single_frame },
        { "get_cooler_status", &get_cooler_status },
        { "set_cooler_state", &set_cooler_state },
        { "get_ccd_temperature", &get_sensor_temperature },
        { "export_config_settings", &export_config_settings },
        { "get_variable_delay_settings", &get_variable_delay_settings },
        { "set_variable_delay_settings", &set_variable_delay_settings },
        { "get_limit_frame", &get_limit_frame },
        { "set_limit_frame", &set_limit_frame },

        // PHD2 extensions
        { "set_extended_protocol", nullptr, EXT_PROTOCOL_UNSPECIFIED, &set_extended_protocol },
        { "get_guiding_period", &get_guiding_period },
        { "set_time_lapse", &set_time_lapse },
        { "set_guide_frame", &set_guide_frame },
        { "get_guide_frame", &get_guide_frame },
        { "get_pixel_size", &get_pixel_size },
        { "set_pixel_size", &set_pixel_size },
        { "get_focal_length", &get_focal_length },
        { "set_focal_length", &set_focal_length },
        { "set_planetary_mode", &set_planetary_mode },
        { "get_cal_settings", &get_cal_settings },
        { "get_mount_coords", nullptr, EXT_PROTOCOL_UNSPECIFIED, &get_mount_coords },
        { "get_site_coords", &get_site_coords },
        { "set_site_coords", &set_site_coords },
        { "get_mount_tracking", nullptr, EXT_PROTOCOL_UNSPECIFIED, &get_mount_tracking },
        { "set_mount_tracking", &set_mount_tracking },
        { "set_surf_mode", &set_surf_mode },
        { "get_surf_mode", &get_surf_mode },
        { "set_frame_export", nullptr, EXT_PROTOCOL_SOLAR3, &set_frame_export },
        { "get_frame_export", &get_frame_export, EXT_PROTOCOL_SOLAR3 },
        { "set_planet_detection", &set_planet_detection, EXT_PROTOCOL_SOLAR3 },
        { "get_planet_state", &get_planet_state, EXT_PROTOCOL_SOLAR3 },
        { "set_planet_thresholds", &set_planet_thresholds, EXT_PROTOCOL_SOLAR3 },
        { "get_process_id", &get_process_id },
        { "set_planet_size", &set_planet_size},
        { "set_cal_step", &set_cal_step },
        { "set_iflink", &set_iflink },
        { "set_iflink_cam", &set_iflink_cam },
        { "get_cal_data", &get_cal_data },

        { "get_drift_measurement", &get_drift_measurement, EXT_PROTOCOL_SOLAR2 },
        { "get_mount_side_of_pier", &get_mount_side_of_pier, EXT_PROTOCOL_SOLAR2 },
        { "get_mount_destination_side_of_pier", &get_mount_destination_side_of_pier, EXT_PROTOCOL_SOLAR2 },
        { "is_parked", &is_parked, EXT_PROTOCOL_SOLAR2 },
        { "park", &park, EXT_PROTOCOL_SOLAR2 },
        { "unpark", &unpark, EXT_PROTOCOL_SOLAR2 },
        { "slew_to_coordinates", &slew_to_coordinates, EXT_PROTOCOL_SOLAR2 },
        { "sync_coordinates", &sync_coordinates, EXT_PROTOCOL_SOLAR2 },
        { "move_axis", &move_axis, EXT_PROTOCOL_SOLAR2 },
        { "get_axis_rates", &get_axis_rates, EXT_PROTOCOL_SOLAR2 },
        { "poll_mount_slewing", &poll_mount_slewing, EXT_PROTOCOL_SOLAR2 },
        { "abort_slew", &abort_slew, EXT_PROTOCOL_SOLAR2 },
        { "get_mount_caps", &get_mount_caps, EXT_PROTOCOL_SOLAR2 },
        { "get_equatorial_system", &get_equatorial_system, EXT_PROTOCOL_SOLAR2 },
        { "does_refraction", &does_refraction, EXT_PROTOCOL_SOLAR2 },
    };

    for (unsigned int i = 0; i < WXSIZEOF(methods); i++)
    {
        if (strcmp(call.method->string_value, methods[i].name) == 0)
        {
            if (methods[i].minProtocol != EXT_PROTOCOL_UNSPECIFIED && protocol < methods[i].minProtocol)
                break;

            if (methods[i].fnWithClient)
                (*methods[i].fnWithClient)(call.response, params, cd);
            else
                (*methods[i].fn)(call.response, params);
            if (id)
            {
                call.response << jrpc_id(id);
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    if (id)
    {
        call.response << jrpc_error(JSONRPC_METHOD_NOT_FOUND, "method not found") << jrpc_id(id);
        return true;
    }
    else
    {
        return false;
    }
}

static void handle_cli_input_complete(wxSocketClient *cli, char *input)
{
    // a dedicated JsonParser instance is used for each line of input since
    // handle_request can recurse if the request causes the event loop to run and we
    // don't want the parser to be reused.
    JsonParser parser;

    if (!parser.Parse(input))
    {
        JRpcCall call(cli, nullptr);
        call.response << jrpc_error(JSONRPC_PARSE_ERROR, parser_error(parser)) << jrpc_id(0);
        dump_response(call);
        do_notify1(cli, call.response);
        return;
    }

    const json_value *root = parser.Root();

    if (root->type == JSON_ARRAY)
    {
        // a batch request

        JAry ary;

        bool found = false;
        json_for_each(req, root)
        {
            JRpcCall call(cli, req);
            if (handle_request(call))
            {
                dump_response(call);
                ary << call.response;
                found = true;
            }
        }

        if (found)
            do_notify1(cli, ary);
    }
    else
    {
        // a single request

        const json_value *const req = root;
        JRpcCall call(cli, req);
        if (handle_request(call))
        {
            dump_response(call);
            do_notify1(cli, call.response);
        }
    }
}

static void handle_cli_input(wxSocketClient *cli)
{
    // Bump refcnt to protect against reentrancy.
    //
    // Some functions like set_connected can cause the event loop to run reentrantly. If the
    // client disconnects before the response is sent and a socket disconnect event is
    // dispatched the client data could be destroyed before we respond.

    ClientDataGuard clidata(cli);

    ClientReadBuf *rdbuf = &clidata->rdbuf;

    wxSocketInputStream sis(*cli);

    while (sis.CanRead())
    {
        if (rdbuf->avail() == 0)
        {
            drain_input(sis);

            JRpcResponse response;
            response << jrpc_error(JSONRPC_INTERNAL_ERROR, "too big") << jrpc_id(0);
            do_notify1(cli, response);

            rdbuf->reset();
            break;
        }
        size_t n = sis.Read(rdbuf->dest, rdbuf->avail()).LastRead();
        if (n == 0)
            break;

        rdbuf->dest += n;

        char *end;
        while ((end = static_cast<char *>(memchr(rdbuf->buf(), '\n', rdbuf->len()))) != nullptr)
        {
            // Move the newline-terminated chunk from the read buffer to a temporary
            // buffer on the stack, and consume the chunk from the read buffer before
            // processing the line. This leaves the read buffer in the correct state to
            // be used again if this function is caller reentrantly.
            char line[ClientReadBuf::SIZE];
            size_t len1 = end - rdbuf->buf();
            memcpy(line, rdbuf->buf(), len1);
            line[len1] = 0;

            char *next = end + 1;
            size_t len2 = rdbuf->dest - next;
            memmove(rdbuf->buf(), next, len2);
            rdbuf->dest = rdbuf->buf() + len2;

            handle_cli_input_complete(cli, line);
        }
    }
}

EventServer::EventServer() : m_configEventDebouncer(nullptr) { }

EventServer::~EventServer() { }

bool EventServer::EventServerStart(unsigned int instanceId)
{
    if (m_serverSocket)
    {
        Debug.AddLine("attempt to start event server when it is already started?");
        return false;
    }

    unsigned int port = 4400 + instanceId - 1;
    wxIPV4address eventServerAddr;
    eventServerAddr.Service(port);
    m_serverSocket = new wxSocketServer(eventServerAddr, wxSOCKET_REUSEADDR);

    if (!m_serverSocket->Ok())
    {
        Debug.Write(wxString::Format("Event server failed to start - Could not listen at port %u\n", port));
        delete m_serverSocket;
        m_serverSocket = nullptr;
        return true;
    }

    m_serverSocket->SetEventHandler(*this, EVENT_SERVER_ID);
    m_serverSocket->SetNotify(wxSOCKET_CONNECTION_FLAG);
    m_serverSocket->Notify(true);

    m_configEventDebouncer = new wxTimer();

    Debug.Write(wxString::Format("event server started, listening on port %u\n", port));

    return false;
}

void EventServer::EventServerStop()
{
    wxMutexLocker lck(m_clientsLock);
    if (!m_serverSocket)
        return;

    // Stop socket events before destroying clients; queued events can outlive
    // ClientData when wxSocket destruction is delayed.
    for (CliSockSet::const_iterator it = m_eventServerClients.begin(); it != m_eventServerClients.end(); ++it)
    {
        (*it)->Notify(false);
    }
    DeletePendingEvents();

    for (CliSockSet::const_iterator it = m_eventServerClients.begin(); it != m_eventServerClients.end(); ++it)
    {
        destroy_client(*it);
    }
    m_eventServerClients.clear();

    delete m_serverSocket;
    m_serverSocket = nullptr;

    delete m_configEventDebouncer;
    m_configEventDebouncer = nullptr;

    Debug.AddLine("event server stopped");
}

void EventServer::OnEventServerEvent(wxSocketEvent& event)
{
    wxSocketServer *server = static_cast<wxSocketServer *>(event.GetSocket());

    if (event.GetSocketEvent() != wxSOCKET_CONNECTION)
        return;

    wxSocketClient *client = static_cast<wxSocketClient *>(server->Accept(false));

    if (!client)
        return;

    Debug.Write(wxString::Format("evsrv: cli %p connect\n", client));

    client->SetEventHandler(*this, EVENT_SERVER_CLIENT_ID);
    client->SetNotify(wxSOCKET_LOST_FLAG | wxSOCKET_INPUT_FLAG);
    client->SetFlags(wxSOCKET_NOWAIT);
    client->Notify(true);
    client->SetClientData(new ClientData(client));

    send_catchup_events(client);

    wxMutexLocker lck(m_clientsLock);
    m_eventServerClients.insert(client);
}

void EventServer::OnEventServerClientEvent(wxSocketEvent& event)
{
    wxSocketClient *cli = static_cast<wxSocketClient *>(event.GetSocket());

    if (event.GetSocketEvent() == wxSOCKET_LOST)
    {
        Debug.Write(wxString::Format("evsrv: cli %p disconnect\n", cli));

        wxMutexLocker lck(m_clientsLock);
        unsigned int const n = m_eventServerClients.erase(cli);
        if (n != 1)
            Debug.AddLine("client disconnected but not present in client set!");

        if (cli == s_frameExportClient)
        {
            s_frameExportClient = nullptr;
            FrameExport::Enable(false);
        }

        destroy_client(cli);
    }
    else if (event.GetSocketEvent() == wxSOCKET_INPUT)
    {
        handle_cli_input(cli);
    }
    else
    {
        Debug.Write(wxString::Format("unexpected client socket event %d\n", event.GetSocketEvent()));
    }
}

void EventServer::NotifyStartCalibration(const Mount *mount)
{
    SIMPLE_NOTIFY_EV(ev_start_calibration(mount));
}

void EventServer::NotifyCalibrationStep(const CalibrationStepInfo& info)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev("Calibrating");

    ev << NVMount(info.mount) << NV("dir", info.direction) << NV("dist", info.dist) << NV("dx", info.dx) << NV("dy", info.dy)
       << NV("pos", info.pos) << NV("step", info.stepNumber);

    if (!info.msg.empty())
        ev << NV("State", info.msg);

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyCalibrationUpdate()
{
    SIMPLE_NOTIFY("CalibrationUpdate");
}

void EventServer::NotifyCalibrationFailed(const Mount *mount, const wxString& msg)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev("CalibrationFailed");
    ev << NVMount(mount) << NV("Reason", msg);

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyCalibrationComplete(const Mount *mount, CalibrationIssueType issue)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    do_notify(m_eventServerClients, ev_calibration_complete(mount, issue));
}

void EventServer::NotifyCalibrationDataFlipped(const Mount *mount)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev("CalibrationDataFlipped");
    ev << NVMount(mount);

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyLooping(unsigned int exposure, const Star *star, const FrameDroppedInfo *info)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev("LoopingExposures");
    ev << NV("Frame", exposure);

    double mass = 0., snr, hfd;
    int err = 0;
    wxString status;

    if (star)
    {
        mass = star->Mass;
        snr = star->SNR;
        hfd = star->HFD;
        err = star->GetError();
    }
    else if (info)
    {
        if (Star::WasFound(static_cast<Star::FindResult>(info->starError)))
        {
            mass = info->starMass;
            snr = info->starSNR;
            hfd = info->starHFD;
        }
        err = info->starError;
        status = info->status;
    }

    if (mass)
    {
        ev << NV("StarMass", mass, 0) << NV("SNR", snr, 2) << NV("HFD", hfd, 2);
    }

    if (err)
        ev << NV("ErrorCode", err);

    if (!status.IsEmpty())
        ev << NV("Status", status);

    if (star && star->IsValid())
    {
        PHD_Point pos(star->X, star->Y);
        ev << NV("StarPos", pos);
    }

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyLoopingStopped()
{
    SIMPLE_NOTIFY("LoopingExposuresStopped");
}

void EventServer::NotifySingleFrameComplete(bool succeeded, const wxString& errorMsg, const SingleExposure& info)
{
    if (m_eventServerClients.empty())
        return;

    Ev ev("SingleFrameComplete");
    ev << NV("Success", succeeded);

    if (!succeeded)
        ev << NV("Error", errorMsg);

    if (info.save)
    {
        ev << NV("Path", info.path);
    }

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyStarSelected(const PHD_Point& pt)
{
    SIMPLE_NOTIFY_EV(ev_star_selected(pt));
}

void EventServer::NotifyStarLost(const FrameDroppedInfo& info)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev("StarLost");

    ev << NV("Frame", info.frameNumber) << NV("Time", info.time, 3) << NV("StarMass", info.starMass, 0)
       << NV("SNR", info.starSNR, 2) << NV("HFD", info.starHFD, 2) << NV("AvgDist", info.avgDist, 2);

    if (info.starError)
        ev << NV("ErrorCode", info.starError);

    if (!info.status.IsEmpty())
        ev << NV("Status", info.status);

    if (!info.state.IsEmpty())
        ev << NV("State", info.state);

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyGuidingStarted()
{
    SIMPLE_NOTIFY_EV(ev_start_guiding());
}

void EventServer::NotifyGuidingStopped()
{
    SIMPLE_NOTIFY("GuidingStopped");
}

void EventServer::NotifyPaused()
{
    SIMPLE_NOTIFY_EV(ev_paused());
}

void EventServer::NotifyResumed()
{
    SIMPLE_NOTIFY("Resumed");
}

void EventServer::NotifyGuideStep(const GuideStepInfo& step)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev("GuideStep");

    ev << NV("Frame", step.frameNumber) << NV("Time", step.time, 3) << NVMount(step.mount) << NV("dx", step.cameraOffset.X, 3)
       << NV("dy", step.cameraOffset.Y, 3) << NV("RADistanceRaw", step.mountOffset.X, 3)
       << NV("DECDistanceRaw", step.mountOffset.Y, 3) << NV("RADistanceGuide", step.guideDistanceRA, 3)
       << NV("DECDistanceGuide", step.guideDistanceDec, 3);

    if (step.durationRA > 0)
    {
        ev << NV("RADuration", step.durationRA)
           << NV("RADirection", step.mount->DirectionStr((GUIDE_DIRECTION) step.directionRA));
    }

    if (step.durationDec > 0)
    {
        ev << NV("DECDuration", step.durationDec)
           << NV("DECDirection", step.mount->DirectionStr((GUIDE_DIRECTION) step.directionDec));
    }

    if (step.mount->IsStepGuider())
    {
        ev << NV("Pos", step.aoPos);
    }

    ev << NV("StarMass", step.starMass, 0) << NV("SNR", step.starSNR, 2) << NV("HFD", step.starHFD, 2)
       << NV("AvgDist", step.avgDist, 2);

    if (step.starError)
        ev << NV("ErrorCode", step.starError);

    if (step.raLimited)
        ev << NV("RALimited", true);

    if (step.decLimited)
        ev << NV("DecLimited", true);

    if (step.starPos.IsValid())
    {
        ev << NV("StarPos", step.starPos);
    }

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifySt4Step(char direction, int duration)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev("St4Step");
    ev << NV("Dir", direction) << NV("Step", duration);

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyGuidingDithered(double dx, double dy)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev("GuidingDithered");
    ev << NV("dx", dx, 3) << NV("dy", dy, 3);

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifySetLockPosition(const PHD_Point& xy)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    do_notify(m_eventServerClients, ev_set_lock_position(xy));
}

void EventServer::NotifyLockPositionLost()
{
    SIMPLE_NOTIFY("LockPositionLost");
}

void EventServer::NotifyLockShiftLimitReached()
{
    SIMPLE_NOTIFY("LockPositionShiftLimitReached");
}

void EventServer::NotifyAppState()
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    do_notify(m_eventServerClients, ev_app_state());
}

void EventServer::NotifySettleBegin()
{
    SIMPLE_NOTIFY("SettleBegin");
}

void EventServer::NotifySettling(double distance, double time, double settleTime, bool starLocked)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev(ev_settling(distance, time, settleTime, starLocked));

    Debug.Write(wxString::Format("evsrv: %s\n", ev.str()));

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifySettleDone(const wxString& errorMsg, int settleFrames, int droppedFrames)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev(ev_settle_done(errorMsg, settleFrames, droppedFrames));

    Debug.Write(wxString::Format("evsrv: %s\n", ev.str()));

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyAlert(const wxString& msg, int type)
{
    wxMutexLocker lck(m_clientsLock);
    if (m_eventServerClients.empty())
        return;

    Ev ev("Alert");
    ev << NV("Msg", msg);

    wxString s;
    switch (type)
    {
    case wxICON_NONE:
    case wxICON_INFORMATION:
    default:
        s = "info";
        break;
    case wxICON_QUESTION:
        s = "question";
        break;
    case wxICON_WARNING:
        s = "warning";
        break;
    case wxICON_ERROR:
        s = "error";
        break;
    }
    ev << NV("Type", s);

    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyClearAlert()
{
    SIMPLE_NOTIFY("ClearAlert");
}

template<typename T>
static void NotifyGuidingParam(wxMutex& lock, const EventServer::CliSockSet& clients, const wxString& name, T val)
{
    wxMutexLocker lck(lock);
    if (clients.empty())
        return;

    Ev ev("GuideParamChange");
    ev << NV("Name", name);
    ev << NV("Value", val);

    do_notify(clients, ev);
}

void EventServer::NotifyGuidingParam(const wxString& name, double val)
{
    ::NotifyGuidingParam(m_clientsLock, m_eventServerClients, name, val);
}

void EventServer::NotifyGuidingParam(const wxString& name, int val)
{
    ::NotifyGuidingParam(m_clientsLock, m_eventServerClients, name, val);
}

void EventServer::NotifyGuidingParam(const wxString& name, bool val)
{
    ::NotifyGuidingParam(m_clientsLock, m_eventServerClients, name, val);
}

void EventServer::NotifyGuidingParam(const wxString& name, const wxString& val)
{
    ::NotifyGuidingParam(m_clientsLock, m_eventServerClients, name, val);
}

void EventServer::NotifyConfigurationChange()
{
    if (m_configEventDebouncer == nullptr || m_configEventDebouncer->IsRunning())
        return;

    wxMutexLocker lck(m_clientsLock);
    Ev ev("ConfigurationChange");
    do_notify(m_eventServerClients, ev);
    m_configEventDebouncer->StartOnce(0);
}

void EventServer::NotifyGearChange()
{
    wxMutexLocker lck(m_clientsLock);
    Ev ev("GearChange");
    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyPlanetaryDetection(bool detected, int points, double score, int radius)
{
    wxMutexLocker lck(m_clientsLock);
    Ev ev("PlanetaryDetection");
    ev << NV("detect", detected);
    ev << NV("points", points);
    ev << NV("score", score);
    ev << NV("radius", radius);
    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyDetectionRequest(int frame)
{
    wxMutexLocker lck(m_clientsLock);
    Ev ev("DetectionRequest");
    ev << NV("frame", frame);
    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyPlanetMetrics(double snr, double mass, int peak)
{
    wxMutexLocker lck(m_clientsLock);
    Ev ev("PlanetMetrics");
    ev << NV("snr", snr);
    ev << NV("mass", mass);
    ev << NV("peak", peak);
    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyMouseClick(PHD_Point& click)
{
    wxMutexLocker lck(m_clientsLock);
    Ev ev("MouseClick");
    ev << click;
    do_notify(m_eventServerClients, ev);
}

void EventServer::NotifyAutoSelect()
{
    SIMPLE_NOTIFY("AutoSelect");
}

void EventServer::NotifyStartCapture()
{
    SIMPLE_NOTIFY("StartCapture");
}
