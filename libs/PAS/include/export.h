/* +---------------------------------------------------------------------------+
*  |                              			                                   |
*  |                                                                           |
*  |                                                                           |
*  | Copyright (c) 2019, - All rights reserved.                                |
*  | Authors:                                                                  |
*  | Released under ___ License.                                               |
*  +---------------------------------------------------------------------------+ */

 // for shared libraries 
 
#ifndef __PAS_EXPORT_
#define __PAS_EXPORT_

#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
#  ifdef PAS
#    define PAS_EXPORT __declspec(dllexport)
#  else
#    define PAS_EXPORT __declspec(dllimport)
#  endif
#else
#  define PAS_EXPORT
#endif

#endif // __PAS_EXPORT_




using namespace std;