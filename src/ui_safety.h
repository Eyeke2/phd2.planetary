/*
 *  Detects wx event-loop reentry from event-server RPC handlers.
 *  In release builds the checker logs and warns, but it does not block
 *  the operation.
 */

#ifndef UI_SAFETY_H_INCLUDED
#define UI_SAFETY_H_INCLUDED

namespace ui_safety {

// Logs unsafe UI operations from RPC paths; debug builds also assert.
void Check(const char *op, const char *file, int line);

// Suppresses user-facing warnings for audited RPC paths that intentionally
// drain wx pending events without entering the OS message loop.
class VettedScope
{
public:
    VettedScope() noexcept;
    ~VettedScope() noexcept;
private:
    VettedScope(const VettedScope&) = delete;
    VettedScope& operator=(const VettedScope&) = delete;
};

} // namespace ui_safety

#define UI_CHECK_SAFETY(op_label)                                              \
    ::ui_safety::Check((op_label), __FILE__, __LINE__)

#define UI_SAFE_UPDATE(window)                                                 \
    do {                                                                       \
        ::ui_safety::Check("wxWindow::Update", __FILE__, __LINE__);            \
        (window)->Update();                                                    \
    } while (0)

#define UI_SAFE_SHOW_MODAL(dialog)                                             \
    (::ui_safety::Check("wxDialog::ShowModal", __FILE__, __LINE__),            \
     (dialog)->ShowModal())

#define UI_SAFE_YIELD()                                                        \
    (::ui_safety::Check("wxYield", __FILE__, __LINE__), wxYield())

#endif // UI_SAFETY_H_INCLUDED
