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

// Numeric tuple for VS_VERSION_INFO FILEVERSION / PRODUCTVERSION.
// Win32 allows only 4 integer components, so the "-solar.N[.M]" sub-revision
// is packed into PHD_VER_REVISION as (SOLAR_MAJOR * 1000 + SOLAR_PATCH):
//   v2.6.14-solar.2    -> 2,6,14,2000
//   v2.6.14-solar.3    -> 2,6,14,3000
//   v2.6.14-solar.3.1  -> 2,6,14,3001
//   v2.6.14-solar.4    -> 2,6,14,4000
// The 1000 multiplier keeps the numeric tuple monotonic across the entire
// -solar.* series and leaves room for up to 999 patches per solar branch.
#define PHD_VER_MAJOR        2
#define PHD_VER_MINOR        6
#define PHD_VER_BUILD        14
#define PHD_VER_SOLAR_MAJOR  5
#define PHD_VER_SOLAR_PATCH  0

// Composite revision component for VS_VERSION_INFO (4th of MAJOR.MINOR.BUILD.REVISION).
// MUST equal PHD_VER_SOLAR_MAJOR * 1000 + PHD_VER_SOLAR_PATCH.
// Kept as a plain integer literal because rc.exe does not evaluate arithmetic
// expressions inside VERSIONINFO FILEVERSION / PRODUCTVERSION fields.
#define PHD_VER_REVISION     5000

// Build-time guard so the literal above can't drift from SOLAR_MAJOR/SOLAR_PATCH.
// rc.exe doesn't define __cplusplus, so this is a no-op for the resource compiler.
#ifdef __cplusplus
#define PHD_VER_REVISION_ (PHD_VER_SOLAR_MAJOR * 1000 + PHD_VER_SOLAR_PATCH)
static_assert(PHD_VER_REVISION == PHD_VER_REVISION_,
              "PHD_VER_REVISION must equal PHD_VER_SOLAR_MAJOR * 1000 + PHD_VER_SOLAR_PATCH");
#endif

// String forms: base version, fork suffix, and combined.
// Use plain ASCII string literals here - phd.h wraps these with _T()
// for wxWidgets, and rc.exe concatenates them directly.
// Keep PHD_VER_SUBSTRING in sync with SOLAR_MAJOR/SOLAR_PATCH:
//   SOLAR_PATCH == 0  -> "-solar.<SOLAR_MAJOR>"
//   SOLAR_PATCH  > 0  -> "-solar.<SOLAR_MAJOR>.<SOLAR_PATCH>"
#define PHD_VER_STRING     "2.6.14"
#define PHD_VER_SUBSTRING  "-solar.5"
#define PHD_VER_FULL       PHD_VER_STRING PHD_VER_SUBSTRING

#endif // PHD_VERSION_H_INCLUDED
