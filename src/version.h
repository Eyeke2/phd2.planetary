/*
 *  version.h
 *  PHD Guiding
 *
 *  Version constants shared between phd.h and phd.rc. Plain C strings
 *  and integer literals only - no _T()/wxT() and no other includes -
 *  so the Windows resource compiler (rc.exe) can read this header.
 *
 *  When cutting a release, bump only this file; both the application
 *  About box / log headers / JSON-RPC version field, and the Windows
 *  resource block (Explorer "Details" tab, installer) stay in sync.
 */

#ifndef PHD_VERSION_H_INCLUDED
#define PHD_VERSION_H_INCLUDED

// Numeric tuple for VS_VERSION_INFO FILEVERSION / PRODUCTVERSION
#define PHD_VER_MAJOR     2
#define PHD_VER_MINOR     6
#define PHD_VER_BUILD     14
#define PHD_VER_REVISION  0

// String forms: base version, fork suffix, and combined.
// Use plain ASCII string literals here - phd.h wraps these with _T()
// for wxWidgets, and rc.exe concatenates them directly.
#define PHD_VER_STRING     "2.6.14"
#define PHD_VER_SUBSTRING  "-solar.2.10"
#define PHD_VER_FULL       PHD_VER_STRING PHD_VER_SUBSTRING

#endif // PHD_VERSION_H_INCLUDED
