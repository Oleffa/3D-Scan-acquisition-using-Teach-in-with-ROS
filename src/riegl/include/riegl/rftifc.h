/* $Id: rftifc.h 226 2009-06-19 12:10:31Z rs $ */

#ifndef RFTIFC_H
#define RFTIFC_H

/*!\file
 * The Riegl File Transfer (DLL) interface definitions.
 */

#ifdef major
#   undef major
#endif
#ifdef minor
#   undef minor
#endif

#ifdef _WIN32
#   ifdef RFTIFC_BUILD_DLL
#       define RFTIFC_API __declspec(dllexport)
#   else
#       define RFTIFC_API __declspec(dllimport)
#   endif
#elif defined(DOXYGEN)
    /*!\brief Public interface tag. */
#   define RFTIFC_API
#else
#   define RFTIFC_API
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/*!\file
 * Basic data types used in the DLL interface.
 */
#ifdef DOXYGEN
    /*!\brief unsigned 8 bit integer */
    typedef IMPLEMENTATION_DEFINED   rftifc_uint8_t;
    /*!\brief signed 8 bit integer */
    typedef IMPLEMENTATION_DEFINED    rftifc_int8_t;
    /*!\brief unsigned 16 bit integer */
    typedef IMPLEMENTATION_DEFINED  rftifc_uint16_t;
    /*!\brief signed 16 bit integer */
    typedef IMPLEMENTATION_DEFINED   rftifc_int16_t;
    /*!\brief unsigned 32 bit integer */
    typedef IMPLEMENTATION_DEFINED  rftifc_uint32_t;
    /*!\brief signed 32 bit integer */
    typedef IMPLEMENTATION_DEFINED   rftifc_int32_t;
    /*!\brief unsigned 64 bit integer */
    typedef IMPLEMENTATION_DEFINED  rftifc_uint64_t;
    /*!\brief signed 64 bit integer */
    typedef IMPLEMENTATION_DEFINED   rftifc_int64_t;
    /*!\brief unsigned largest available bitsize integer */
    typedef IMPLEMENTATION_DEFINED rftifc_uintmax_t;
    /*!\brief signed largest available bitsize integer */
    typedef IMPLEMENTATION_DEFINED  rftifc_intmax_t;
    /*!\brief 32 bit floating point */
    typedef IMPLEMENTATION_DEFINED rftifc_float32_t;
#else
/* The following typedefs might need configuration... */
#   if defined(__MINGW32__) || defined(__GNUC__)
#   include <stdint.h>
    typedef uint8_t   rftifc_uint8_t;
    typedef int8_t    rftifc_int8_t;
    typedef uint16_t  rftifc_uint16_t;
    typedef int16_t   rftifc_int16_t;
    typedef uint32_t  rftifc_uint32_t;
    typedef int32_t   rftifc_int32_t;
    typedef uint64_t  rftifc_uint64_t;
    typedef int64_t   rftifc_int64_t;
    typedef uintmax_t rftifc_uintmax_t;
    typedef intmax_t  rftifc_intmax_t;
    typedef float rftifc_float32_t;
#   elif defined(_MSC_VER)
    typedef unsigned __int8  rftifc_uint8_t;
    typedef unsigned __int16 rftifc_uint16_t;
    typedef unsigned __int32 rftifc_uint32_t;
    typedef unsigned __int64 rftifc_uint64_t;
    typedef __int8           rftifc_int8_t;
    typedef __int16          rftifc_int16_t;
    typedef __int32          rftifc_int32_t;
    typedef __int64          rftifc_int64_t;
    typedef float            rftifc_float32_t;
#   endif
#endif

/*---------------------------------------------------------------------------*/
#ifdef DOXYGEN
/*!\brief a handle to a stream of 3d pointcloud data */
typedef IMPLEMENTATION_DEFINED rftifc_session_handle;
#else
struct rftifc_session_impl;
typedef struct rftifc_session_impl* rftifc_session;
#endif

/*!\brief A zero delimited string */
typedef char* rftifc_sz;

/*!\brief A zero delimited constant string */
typedef const char* rftifc_csz;

/*!\brief A boolean value */
typedef int rftifc_bool;

/**************************************************************************//**
 *  Get version number from library.
 *
 *  This version number usually is different from the version number of the
 *  distribution set of the library. This version number is about the API
 *  of the shared object (DLL).
 *  A change in major version number generally indicates a breaking change
 *  in the API or semantics. A change in minor version usually does not
 *  require a change of user code.
 * \param major major version number
 * \param minor minor version number
 * \param build build number (revision control)
 * \return 0 for succes, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_get_library_version(
    rftifc_uint16_t* major
    , rftifc_uint16_t* minor
    , rftifc_uint16_t* build
);

/**************************************************************************//**
 *  Get extended version information.
 *
 *  The build_version allows traceability to the SCM system.
 * The build tag contains additional information about the build.
 * \param build_version [out] SCM build version
 * \param build_tag [out] additional build information
 * \return 0 for succes, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_get_library_info(
    rftifc_csz *build_version
    , rftifc_csz *build_tag
);

/**************************************************************************//**
 * Create a file transfer session.
 *
 * The create_session function returns a handle that references the session.
 * \param s [out] a handle identifying this particular session.
 * \return 0 for success, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_construct_session(
    rftifc_session* s
);

/**************************************************************************//**
 * Close a session.
 *
 * This function must be called when done with the ssession to release the
 * resources associated with the handle.
 * \param s [in] the ssession heandle that has been returned from construct.
 * \return 0 for success, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_destruct_session(
    rftifc_session s
);

/**************************************************************************//**
 * Get last error message.
 *
 * A failing function (indicated by !=0 return) usually will set an error
 * description string, that can be retrieved with this function.
 * \param s [in] the session handle
 * \param message [ const out] The error message
 * \return 0 for succes, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_get_last_error(
    rftifc_session s
    , rftifc_csz *message_buffer
);

/**************************************************************************//**
 * Establish a session to a scanner.
 *
 * The connect function will locate the scanner by means of the authority
 * information, which may be given as "host:port", where host and port
 * either are in text or numeric forms.
 * \param s [in] the session handle
 * \param authority [in] the address of the scanner
 * \return 0 for success, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_connect(
    rftifc_session s
    , rftifc_csz authority
);

/**************************************************************************//**
 * Copy a file from the scanner to the local file system.
 *
 * The get_file function is resumable from a previous partly succesful
 * transfer. The turn off resume, just delete the .part and .resume
 * files before starting the transfer.
 * \param s [in] the session handle
 * \param count [out] the number of transfered bytes (may be smaller than
 *        file size in case of resume mode, or 0 if file already existed).
 * \param remote [in] the remote file name
 * \param local  [in] the local filename
 * \return 0 for success, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_get_file(
    rftifc_session s
    , rftifc_uint64_t* count
    , rftifc_csz remote
    , rftifc_csz local
);

/**************************************************************************//**
 * Copy a file from the local file system to the scanner.
 *
 * The put_file function transfers a file to the scanner. In case a file
 * with same name aleardy exists, it is overwritten. (put_file is not
 * resumable)
 * \param s [in] the session handle
 * \param count [out] the number of transfered bytes
 * \param local  [in] the local filename
 * \param remote [in] the remote file name
 * \return 0 for success, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_put_file(
    rftifc_session s
    , rftifc_uint64_t* count
    , rftifc_csz local
    , rftifc_csz remote
);

/**************************************************************************//**
 * Get status of file(s)
 *
 * The stat function retrieves status information about files. This is a
 * two phase procedure:
 * 1) The stat function is used to set a file pattern
 * 2) The next_stat function is used to iterate of the results.
 * \param s [in] the session handle
 * \param pattern [in] the file name pattern (see also: unix file name globing)
 * \return 0 for success, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_stat(
    rftifc_session s
    , rftifc_csz pattern
);

/**************************************************************************//**
 * Get (next) status of file(s)
 *
 * The next_stat function retrieves information about files that have been
 * selected with a pattern by a call to the stat function. The end of
 * available data is signalled with time and path set to a null pointer.
 *
 * \param s [in] the session handle
 * \param is_dir [out] true if path is a directory
 * \param size [out] size of the file in octets
 * \param time [out] last modified time
 * \param path [out] the path
 * \return 0 for success, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_next_stat(
    rftifc_session s
    , rftifc_bool *is_dir
    , rftifc_uint64_t *size
    , rftifc_csz *time
    , rftifc_csz *path
);

/**************************************************************************//**
 * Create directory on remote
 *
 * The mkdir function creates a directory on the remote side.
 * \param s [in] the session handle
 * \param remote [in] the name of the directory to create
 * \return 0 for success, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_mkdir(
    rftifc_session s
    , rftifc_csz remote
);

/**************************************************************************//**
 * Remove a file or directory on the remote side
 *
 * The rm function will remove a file or directory.
 * \param s [in] the session handle
 * \param remote [in] the name of the file or directory
 * \param recursive [in] perform the operation recursively
 * \return 0 for success, !=0 for failure
 *****************************************************************************/
RFTIFC_API int
rftifc_rm(
    rftifc_session s
    , rftifc_csz remote
    , rftifc_bool recursive
);


#ifdef __cplusplus
}
#endif

#endif /* RFTIFC_H */
