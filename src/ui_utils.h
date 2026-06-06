/*
 *  ui_utils.h
 *  PHD Guiding
 *
 *  Small wx UI helpers for avoiding redundant control updates.
 */

#ifndef UI_UTILS_H_INCLUDED
#define UI_UTILS_H_INCLUDED

#include <wx/control.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/tooltip.h>

inline void UpdateLabel(wxControl *c, const wxString& v)
{
    if (c && c->GetLabel() != v)
    {
        c->SetLabel(v);
        c->Refresh();
    }
}

inline void UpdateLabel(wxStaticText *c, const wxString& v)
{
    if (c && c->GetLabel() != v)
    {
        c->SetLabel(v);
        c->Refresh();
    }
}

inline void UpdateValue(wxTextCtrl *c, const wxString& v)
{
    // ChangeValue (not SetValue) suppresses wxEVT_TEXT while refreshing.
    if (c && c->GetValue() != v)
        c->ChangeValue(v);
}

// Works for any control with bool GetValue()/SetValue(bool) -- wxCheckBox,
// wxToggleButton, wxBitmapToggleButton, wxRadioButton, etc.
template<typename Ctrl>
inline void UpdateBoolValue(Ctrl *c, bool v)
{
    if (c && c->GetValue() != v)
        c->SetValue(v);
}

inline void UpdateEnable(wxWindow *c, bool v)
{
    if (c && c->IsEnabled() != v)
        c->Enable(v);
}

inline void UpdateToolTip(wxWindow *c, const wxString& v)
{
    if (c && c->GetToolTipText() != v)
        c->SetToolTip(v);
}

inline void DismissToolTip()
{
    wxToolTip::Enable(false);
    wxToolTip::Enable(true);
}

#endif // UI_UTILS_H_INCLUDED
