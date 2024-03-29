
// -*- C++ -*-
// $Id$
// Definition for Win32 Export directives.
// This file is generated automatically by generate_export_file.pl ASIOLib
// ------------------------------
#ifndef ASIOLIB_EXPORT_H
#define ASIOLIB_EXPORT_H

//#include "ace/config-all.h"

//#if defined (ACE_AS_STATIC_LIBS) && !defined (ASIOLIB_HAS_DLL)
//#  define ASIOLIB_HAS_DLL 0
//#endif /* ACE_AS_STATIC_LIBS && ASIOLIB_HAS_DLL */

//#if !defined (ASIOLIB_HAS_DLL)
//#  define ASIOLIB_HAS_DLL 1
//#endif /* ! ASIOLIB_HAS_DLL */

#if defined (ASIOLIB_HAS_DLL) && (ASIOLIB_HAS_DLL == 1)
#  if defined (ASIOLIB_BUILD_DLL)
#    define ASIOLib_Export ACE_Proper_Export_Flag
#    define ASIOLIB_SINGLETON_DECLARATION(T) ACE_EXPORT_SINGLETON_DECLARATION (T)
#    define ASIOLIB_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_EXPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  else /* ASIOLIB_BUILD_DLL */
#    define ASIOLib_Export ACE_Proper_Import_Flag
#    define ASIOLIB_SINGLETON_DECLARATION(T) ACE_IMPORT_SINGLETON_DECLARATION (T)
#    define ASIOLIB_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_IMPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  endif /* ASIOLIB_BUILD_DLL */
#else /* ASIOLIB_HAS_DLL == 1 */
#  define ASIOLib_Export
#  define ASIOLIB_SINGLETON_DECLARATION(T)
#  define ASIOLIB_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#endif /* ASIOLIB_HAS_DLL == 1 */

// Set ASIOLIB_NTRACE = 0 to turn on library specific tracing even if
// tracing is turned off for ACE.
#if !defined (ASIOLIB_NTRACE)
#  if (ACE_NTRACE == 1)
#    define ASIOLIB_NTRACE 1
#  else /* (ACE_NTRACE == 1) */
#    define ASIOLIB_NTRACE 0
#  endif /* (ACE_NTRACE == 1) */
#endif /* !ASIOLIB_NTRACE */

#if (ASIOLIB_NTRACE == 1)
#  define ASIOLIB_TRACE(X)
#else /* (ASIOLIB_NTRACE == 1) */
#  if !defined (ACE_HAS_TRACE)
#    define ACE_HAS_TRACE
#  endif /* ACE_HAS_TRACE */
#  define ASIOLIB_TRACE(X) ACE_TRACE_IMPL(X)
//#  include "ace/Trace.h"
#endif /* (ASIOLIB_NTRACE == 1) */

#endif /* ASIOLIB_EXPORT_H */

// End of auto generated file.
