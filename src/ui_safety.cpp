/*
 *  ui_safety.cpp
 *  PHD Guiding
 *
 *  Implementation of ::ui_safety::Check. See ui_safety.h for the rationale
 *  and the threading-rule comment block above handle_request() in
 *  event_server.cpp for the deadlock class this guards against.
 */

#include "phd.h"
#include "ui_safety.h"
#include "event_server.h"

namespace ui_safety {

// Thread-local depth counter incremented while a VettedScope is alive.
// Read by Check() to decide whether to suppress the noisy paths (the
// debug-build wxFAIL_MSG and the status-bar alert). Depth (rather than
// bool) so vetted scopes nest correctly when one helper that opens a
// VettedScope calls another that does the same.
static thread_local int s_vettedDepth = 0;

VettedScope::VettedScope() noexcept { ++s_vettedDepth; }
VettedScope::~VettedScope() noexcept { --s_vettedDepth; }

void Check(const char *op, const char *file, int line)
{
    // Fast path: if no RPC is in flight on this thread, the operation is
    // safe regardless of what it does. RPC handlers run on the UI thread,
    // and EventServer::InRpcCall() is a thread_local lookup -- single
    // load, no atomic, no lock.
    if (!EventServer::InRpcCall())
        return;

    // We are inside an RPC handler. The operation about to happen could
    // pump the event loop and reach wx's internal hit-test, deadlocking
    // against the foreign client process. Always log so the incident is
    // recoverable from any debug log a user submits. The "(vetted)"
    // marker distinguishes audited call sites (e.g. StopWorkerThread,
    // which uses ProcessPendingEvents instead of wxYield) from the
    // genuinely-unsafe ones we want to fix.
    const bool vetted = s_vettedDepth > 0;
    Debug.Write(wxString::Format(
        "UI-SAFETY: %s '%s' invoked from event-server RPC handler at %s:%d\n",
        vetted ? "(vetted)" : "unsafe", op, file, line));

    // Vetted scope: the audit asserts this site does not actually pump
    // the OS message queue, so neither the developer-facing assertion
    // dialog nor the user-facing status-bar warning is appropriate. Skip
    // both. The debug-log entry above remains for forensic value.
    if (vetted)
        return;

#ifndef NDEBUG
    // Debug builds: surface the wx assertion dialog so a developer running
    // under a debugger gets a stack to inspect. wxFAIL_MSG is non-fatal --
    // the user can choose Continue and the soft-alert path below still
    // runs. We do not use wxASSERT(false, ...) here because that would
    // add a confusing 'false' literal to the message; wxFAIL_MSG conveys
    // intent more clearly.
    wxFAIL_MSG(wxString::Format(
        "UI-SAFETY: unsafe '%s' from RPC handler (%s:%d) -- see ui_safety.h", op, file, line));
#endif

    // Soft-fail visible cue (both build types): a status-bar message,
    // posted via wxQueueEvent so it is delivered AFTER the current RPC
    // handler returns. We must not call MyFrame::Alert() or any other
    // synchronous UI primitive directly from here -- that would re-enter
    // the very situation we are warning about.
    //
    // Rate-limited to one message per 60 seconds so a misbehaving client
    // hammering an unsafe RPC verb cannot spam the status bar.
    static wxLongLong s_lastAlertMs = 0;
    wxLongLong nowMs = wxGetLocalTimeMillis();
    if (pFrame && (nowMs - s_lastAlertMs).GetValue() > 60000)
    {
        s_lastAlertMs = nowMs;
        wxThreadEvent *evt = new wxThreadEvent(wxEVT_THREAD, SET_STATUS_TEXT_EVENT);
        evt->SetString(wxString::Format(
            _("UI-safety warning: unsafe '%s' from network request -- see debug log"),
            op));
        wxQueueEvent(pFrame, evt);
    }

    // Intentionally fall through: the caller proceeds with the operation.
    // Skipping it would be unpredictable from the user's point of view --
    // a setting silently not applied, a redraw not happening -- whereas
    // proceeding only risks a bad outcome on a particular cursor position
    // at the wrong instant. We'd rather log + warn than silently corrupt.
}

} // namespace ui_safety
