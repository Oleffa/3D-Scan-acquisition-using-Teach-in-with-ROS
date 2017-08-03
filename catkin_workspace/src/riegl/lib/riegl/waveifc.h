/* $Id: waveifc.h 74 2008-10-14 14:35:31Z rs $*/
/* This is the interface to V08WAVE.
 */

#ifndef WAVEIFC_HPP
#define WAVEIFC_HPP

#ifdef major
#   undef major
#endif
#ifdef minor
#   undef minor
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef _WIN32
#   ifdef WAVEIFC_BUILD_DLL
#       define WAVEIFC_API __declspec(dllexport)
#   else
#       define WAVEIFC_API __declspec(dllimport)
#   endif
#else
#   define WAVEIFC_API
#endif

//-----------------------------------------------------------------------------
WAVEIFC_API int
waveifc_get_library_version
(
    unsigned short* major
    , unsigned short* minor
    , unsigned short* build
);
//-----------------------------------------------------------------------------
WAVEIFC_API int
waveifc_get_ridataspec_version
(
    unsigned short* major
    , unsigned short* minor
    , unsigned short* build
);
//-----------------------------------------------------------------------------
#define WAVEIFC_HANDLE void*
#define WAVEIFC_INVALID_HANDLE_VALUE 0
//-----------------------------------------------------------------------------
WAVEIFC_API int
waveifc_open
(
    WAVEIFC_HANDLE* r
    , char* url
);
//-----------------------------------------------------------------------------
WAVEIFC_API int
waveifc_close
(
    WAVEIFC_HANDLE h
);
//-----------------------------------------------------------------------------
WAVEIFC_API int
waveifc_cancel
(
    WAVEIFC_HANDLE h
);
//-----------------------------------------------------------------------------
WAVEIFC_API int
waveifc_get_last_error
(
    WAVEIFC_HANDLE h
    , char* message_buffer
    , int message_buffer_size
    , int* message_size
);
//-----------------------------------------------------------------------------
WAVEIFC_API int
waveifc_get_record_id
(
    WAVEIFC_HANDLE h
    , unsigned short* main
    , unsigned short* sub
);
//-----------------------------------------------------------------------------
WAVEIFC_API int
waveifc_get_wave
(
    WAVEIFC_HANDLE h
    , void* wave_buffer
    , int wave_buffer_size
    , int* wave_record_size
);

#ifdef __cplusplus
}
#endif

#endif /* WAVEIFC_H */
