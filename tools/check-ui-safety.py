#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# tools/check-ui-safety.py
#
# Static check that flags wx operations known to pump the event loop
# reentrantly (and therefore risk the cross-process SendMessage deadlock
# documented above handle_request() in src/event_server.cpp). Run as part
# of the pre-commit hook by pre-commit.py.
#
# Currently a WARN-ONLY check: the script always exits 0, even when
# violations are found, so it does not block commits. Promote to blocking
# (return non-zero from main()) after the codebase has been audited and
# the warnings stop appearing on routine commits.
#
# Detection strategy: scan the lines being ADDED in this commit (the '+'
# lines from `git diff --cached -U0`) for the forbidden patterns. We do
# not flag context lines or pre-existing code -- only what this commit
# is introducing. That keeps the noise floor low and aligns the check
# with the actual deadlock-class regressions we want to prevent.
#
# The patterns flagged are conservative: any literal `Update()`,
# `wxYield(`, `wxSafeYield(`, `ShowModal(`. False positives (an
# unrelated method also named Update, for example) are accepted as the
# cost of catching the real cases; the ratio in PHD2 today is
# overwhelmingly the latter.

from __future__ import print_function

import os
import re
import subprocess
import sys


# (regex, label, suggested replacement) tuples. Regex anchors to word
# boundaries to avoid matching e.g. "myUpdate()" or "fooShowModal()".
PATTERNS = [
    (re.compile(r"\bUpdate\s*\(\s*\)"),    "wxWindow::Update()", "UI_SAFE_UPDATE(window)"),
    (re.compile(r"\bwxYield\s*\("),        "wxYield(...)",       "UI_SAFE_YIELD()"),
    (re.compile(r"\bwxSafeYield\s*\("),    "wxSafeYield(...)",   "UI_SAFE_YIELD()"),
    (re.compile(r"\bShowModal\s*\("),      "wxDialog::ShowModal(...)", "UI_SAFE_SHOW_MODAL(dialog)"),
    (re.compile(r"\bwxApp\s*\(\s*\)\s*\.\s*Yield\s*\("), "wxApp::Yield(...)", "UI_SAFE_YIELD()"),
]

# Exemption regexes -- a line matching ANY of these is skipped before the
# patterns above are tried. Use sparingly; each entry is a known false
# positive for which the deadlock class does not apply.
#
#   m_mgr.Update() is wxAuiManager::Update(), which recomputes pane layout
#   without a cross-process hit-test; safe and unrelated to wxWindow::Update().
#   See the comment above the m_mgr declaration in src/myframe.h for the
#   full rationale.
EXEMPTION_PATTERNS = [
    re.compile(r"\bm_mgr\s*[.->]+\s*Update\s*\(\s*\)"),
]

# File extensions worth scanning -- C/C++ only. Also scan headers because
# inline wrappers can introduce the calls there.
INTERESTING_EXTS = (".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".hxx")

# Files that intentionally define or document the very things we flag.
# Skipping them avoids self-warnings.
EXEMPT_BASENAMES = {
    "ui_safety.h",
    "ui_safety.cpp",
    "event_server.cpp",  # contains the doc comment block listing the patterns
}


def is_interesting(path):
    base = os.path.basename(path)
    if base in EXEMPT_BASENAMES:
        return False
    return path.lower().endswith(INTERESTING_EXTS)


def staged_added_lines(filename):
    """Yield (line_no_in_new_file, line_text) for each line ADDED to filename
    in the current staging area, using git's unified-diff hunk headers to
    track line numbers. Lines starting with '+++' are skipped (file headers).
    """
    try:
        out = subprocess.check_output(
            ["git", "diff", "--cached", "-U0", "--", filename],
            stderr=subprocess.STDOUT,
        )
    except subprocess.CalledProcessError as e:
        # If git fails (e.g. file just deleted) treat as no additions.
        sys.stderr.write("check-ui-safety: git diff failed for {0}: {1}\n".format(
            filename, e.output))
        return

    if not out:
        return

    text = out.decode("utf-8", errors="replace")
    cur_new_lineno = 0
    in_hunk = False
    hunk_re = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@")

    for raw in text.splitlines():
        if raw.startswith("+++") or raw.startswith("---"):
            continue
        m = hunk_re.match(raw)
        if m:
            cur_new_lineno = int(m.group(1))
            in_hunk = True
            continue
        if not in_hunk:
            continue
        if raw.startswith("+"):
            yield cur_new_lineno, raw[1:]
            cur_new_lineno += 1
        elif raw.startswith("-"):
            # Removed line -- does not advance the new-file line counter.
            continue
        else:
            # Context line. With -U0 we should not see these, but be safe.
            cur_new_lineno += 1


def check_file(filename, out):
    """Scan the staged additions in filename. Append (filename, lineno,
    label, suggestion, snippet) tuples to `out` for each match."""
    for lineno, text in staged_added_lines(filename):
        # Strip trailing whitespace + CR (file may be CRLF).
        snippet = text.rstrip("\r\n").rstrip()
        # Skip pure-comment lines so we do not trip on doc comments that
        # mention the forbidden names (e.g. event_server.cpp's rule block,
        # though that file is also in EXEMPT_BASENAMES).
        stripped = snippet.lstrip()
        if stripped.startswith("//") or stripped.startswith("*") or stripped.startswith("/*"):
            continue
        # Skip lines that match a known-safe exemption (e.g. m_mgr.Update()
        # which is wxAuiManager::Update(), not wxWindow::Update()).
        if any(rx.search(snippet) for rx in EXEMPTION_PATTERNS):
            continue
        for regex, label, suggestion in PATTERNS:
            if regex.search(snippet):
                out.append((filename, lineno, label, suggestion, snippet))
                break  # one warning per added line is plenty


def staged_files():
    out = subprocess.check_output(
        ["git", "diff", "--cached", "--name-only", "--diff-filter=AM"]
    )
    text = out.decode("utf-8", errors="replace")
    return [f for f in text.splitlines() if is_interesting(f)]


def main(argv):
    # Allow either an explicit file list (for ad-hoc invocation) or fall
    # back to whatever git says is staged.
    if len(argv) > 1:
        files = [f for f in argv[1:] if is_interesting(f)]
    else:
        try:
            files = staged_files()
        except subprocess.CalledProcessError as e:
            sys.stderr.write("check-ui-safety: cannot list staged files: {0}\n".format(e))
            return 0  # warn-only

    findings = []
    for f in files:
        check_file(f, findings)

    if findings:
        sys.stderr.write(
            "\ncheck-ui-safety: WARNING -- {0} potentially unsafe UI call(s) "
            "introduced in this commit:\n".format(len(findings)))
        for fname, lineno, label, suggestion, snippet in findings:
            sys.stderr.write("  {0}:{1}: {2}\n".format(fname, lineno, label))
            sys.stderr.write("    suggestion: replace with {0}\n".format(suggestion))
            sys.stderr.write("    line:       {0}\n".format(snippet.strip()))
        sys.stderr.write(
            "\nIf this code path can be reached from an event-server RPC "
            "handler, the\ncall can deadlock against a remote client. See "
            "src/ui_safety.h and the\nthreading-rule comment block above "
            "handle_request() in src/event_server.cpp.\n"
            "\nThis check is currently warn-only and does not block the commit.\n\n")

    # WARN-ONLY: always succeed.
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
