# $Id: rivlib-config.cmake 489 2010-09-23 11:17:10Z rs $
# This is the CMake project configuration file for using RiVLib.
#
#           Copyright (c) 2010 Riegl Laser Measurement Systems.

# The authors hereby  grant permission  to use,  and copy this software  for the
# purpose  of enhancing the useability  of  Riegl Laser Measurement Systems GmbH
# instruments you own. You may NOT distribute or modify the software for the use
# in commercial applications without the written consent of RLMS.
#
# In any case copyright notices and  this notice must be  retained in all copies
# verbatim in any distributions.

# IN  NO EVENT SHALL  THE AUTHORS OR  DISTRIBUTORS BE  LIABLE TO  ANY PARTY  FOR
# DIRECT, INDIRECT, SPECIAL,  INCIDENTAL, OR  CONSEQUENTIAL DAMAGES  ARISING OUT
# OF THE USE OF THIS SOFTWARE, ITS  DOCUMENTATION,  OR ANY DERIVATIVES  THEREOF,
# EVEN IF THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHORS AND DISTRIBUTORS SPECIFICALLY DISCLAIM  ANY WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES  OF MERCHANTABILITY,  FITNESS FOR A
# PARTICULAR PURPOSE,  AND  NON-INFRINGEMENT. THIS SOFTWARE  IS  PROVIDED  ON AN
# "AS IS" BASIS, AND THE AUTHORS  AND DISTRIBUTORS HAVE NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

# == Using RiVLib library components: ==
#
#   set(RiVLib_USE_STATIC_RUNTIME   ON) # optional if you need static runtime
#
#   find_package( RiVLib COMPONENTS scanlib ... ) # specify needed components
#                                                 # one ore more from
#                                                 # scanlib
#                                                 # ctrllib
#                                                 # scanifc
#                                                 # ctrlifc
#   add_executable( my_app
#        my_app.cpp
#   )
#   target_link_libraries( my_app
#       ${RiVLib_SCANLIB_LIBRARIES}
#   )
#
#
# == Variables defined by this module: ==
#
# RiVLib_${COMPONENT}_LIBRARY_DEBUG
# RiVLib_${COMPONENT}_LIBRARY_RELEASE
# RiVLib_${COMPONENT}_LIBRARY
# RiVLib_${COMPONENT}_RUNTIME_LIBRARIES
#
# RiVLib_LIBRARIES
# RiVLib_INCLUDE_DIRS
# RiVLib_LIBRARY_DIRS
#


SET( _boost_vns_ver "1_43_0")

IF(RiVLib_USE_STATIC_RUNTIME)
    SET(_rivlib_release_tag "-s")
    SET(_rivlib_debug_tag "-sg")
ELSE()
    SET(_rivlib_release_tag "")
    SET(_rivlib_debug_tag "-g")
ENDIF()

SET(RiVLib_FOUND TRUE)

FOREACH(COMPONENT ${RiVLib_FIND_COMPONENTS})
    STRING(TOUPPER ${COMPONENT} UPPERCOMPONENT)
    SET( RiVLib_${UPPERCOMPONENT}_LIBRARY "RiVLib_${UPPERCOMPONENT}_LIBRARY-NOTFOUND" )
    SET( RiVLib_${UPPERCOMPONENT}_LIBRARY_RELEASE "RiVLib_${UPPERCOMPONENT}_LIBRARY_RELEASE-NOTFOUND" )
    SET( RiVLib_${UPPERCOMPONENT}_LIBRARY_DEBUG "RiVLib_${UPPERCOMPONENT}_LIBRARY_DEBUG-NOTFOUND")

    FIND_LIBRARY(RiVLib_${UPPERCOMPONENT}_LIBRARY_RELEASE
        NAMES   lib${COMPONENT}-mt${_rivlib_release_tag}
                ${COMPONENT}-mt${_rivlib_release_tag}
                ${COMPONENT}-mt
        HINTS  ${RiVLib_DIR}/lib
    )

    FIND_LIBRARY(RiVLib_${UPPERCOMPONENT}_LIBRARY_DEBUG
        NAMES   lib${COMPONENT}-mt${_rivlib_debug_tag}
                ${COMPONENT}-mt${_rivlib_debug_tag}
                ${COMPONENT}-mt
        HINTS  ${RiVLib_DIR}/lib
    )

    MARK_AS_ADVANCED(
        RiVLib_${UPPERCOMPONENT}_LIBRARY_RELEASE
        RiVLib_${UPPERCOMPONENT}_LIBRARY_DEBUG
        RiVLib_${UPPERCOMPONENT}_LIBRARY
    )
ENDFOREACH(COMPONENT)

IF( RiVLib_SCANLIB_LIBRARY_RELEASE )
    FOREACH(_boostlib_ system thread date_time regex)
        FIND_LIBRARY(_boostlib_found_
            NAMES libboost_${_boostlib_}-mt${_rivlib_release_tag}-${_boost_vns_ver}-vns
                  boost_${_boostlib_}-mt${_rivlib_release_tag}-${_boost_vns_ver}-vns
            HINTS  ${RiVLib_DIR}/lib
        )
        IF (_boostlib_found_)
            LIST(APPEND RiVLib_SCANLIB_LIBRARY_RELEASE optimized ${_boostlib_found_})
        ENDIF()
        UNSET(_boostlib_found_ CACHE)
    ENDFOREACH()
    IF (MINGW OR MSVC)
        LIST(APPEND RiVLib_SCANLIB_LIBRARY_RELEASE optimized ws2_32)
    ENDIF(MINGW OR MSVC)
    IF(UNIX)
        LIST(APPEND RiVLib_SCANLIB_LIBRARY_DEBUG optimized pthread)
    ENDIF()
ENDIF()

IF( RiVLib_SCANLIB_LIBRARY_DEBUG )
    FOREACH(_boostlib_ system thread date_time regex)
        FIND_LIBRARY(_boostlib_found_
            NAMES libboost_${_boostlib_}-mt${_rivlib_debug_tag}-${_boost_vns_ver}-vns
                  libboost_${_boostlib_}-mt-${_boost_vns_ver}-vns
                  boost_${_boostlib_}-mt${_rivlib_debug_tag}-${_boost_vns_ver}-vns
                  boost_${_boostlib_}-mt-${_boost_vns_ver}-vns
            HINTS  ${RiVLib_DIR}/lib
        )
        IF (_boostlib_found_)
            LIST(APPEND RiVLib_SCANLIB_LIBRARY_DEBUG debug ${_boostlib_found_})
        ENDIF()
        UNSET(_boostlib_found_ CACHE)
    ENDFOREACH()
    IF (MINGW OR MSVC)
        LIST(APPEND RiVLib_SCANLIB_LIBRARY_DEBUG debug ws2_32)
    ENDIF(MINGW OR MSVC)
    IF(UNIX)
        LIST(APPEND RiVLib_SCANLIB_LIBRARY_DEBUG debug pthread)
    ENDIF()
ENDIF()

IF( RiVLib_CTRLLIB_LIBRARY_RELEASE )
    IF (MINGW OR MSVC)
        LIST(APPEND RiVLib_CTRLLIB_LIBRARY_RELEASE optimized ws2_32)
    ENDIF()
ENDIF()

IF( RiVLib_CTRLLIB_LIBRARY_DEBUG )
    IF (MINGW OR MSVC)
        LIST(APPEND RiVLib_CTRLLIB_LIBRARY_DEBUG debug ws2_32)
    ENDIF()
ENDIF()


FOREACH(COMPONENT ${RiVLib_FIND_COMPONENTS})
    STRING(TOUPPER ${COMPONENT} basename)

    IF (RiVLib_${basename}_LIBRARY_DEBUG AND RiVLib_${basename}_LIBRARY_RELEASE)
      # if the generator supports configuration types then set
      # optimized and debug libraries, or if the CMAKE_BUILD_TYPE has a value
      IF (CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE)
        SET(RiVLib_${basename}_LIBRARY optimized ${RiVLib_${basename}_LIBRARY_RELEASE} debug ${RiVLib_${basename}_LIBRARY_DEBUG})
      ELSE()
        # if there are no configuration types and CMAKE_BUILD_TYPE has no value
        # then just use the release libraries
        SET(RiVLib_${basename}_LIBRARY ${RiVLib_${basename}_LIBRARY_RELEASE} )
      ENDIF()
      # FIXME: This probably should be set for both cases
      SET(RiVLib_${basename}_LIBRARIES optimized ${RiVLib_${basename}_LIBRARY_RELEASE} debug ${RiVLib_${basename}_LIBRARY_DEBUG})
    ENDIF()

    # if only the release version was found, set the debug variable also to the release version
    IF (RiVLib_${basename}_LIBRARY_RELEASE AND NOT RiVLib_${basename}_LIBRARY_DEBUG)
      SET(RiVLib_${basename}_LIBRARY_DEBUG ${RiVLib_${basename}_LIBRARY_RELEASE})
      SET(RiVLib_${basename}_LIBRARY       ${RiVLib_${basename}_LIBRARY_RELEASE})
      SET(RiVLib_${basename}_LIBRARIES     ${RiVLib_${basename}_LIBRARY_RELEASE})
    ENDIF()

    # if only the debug version was found, set the release variable also to the debug version
    IF (RiVLib_${basename}_LIBRARY_DEBUG AND NOT RiVLib_${basename}_LIBRARY_RELEASE)
      SET(RiVLib_${basename}_LIBRARY_RELEASE ${RiVLib_${basename}_LIBRARY_DEBUG})
      SET(RiVLib_${basename}_LIBRARY         ${RiVLib_${basename}_LIBRARY_DEBUG})
      SET(RiVLib_${basename}_LIBRARIES       ${RiVLib_${basename}_LIBRARY_DEBUG})
    ENDIF()
    IF (RiVLib_${basename}_LIBRARY)
      set(RiVLib_${basename}_LIBRARY ${RiVLib_${basename}_LIBRARY} CACHE FILEPATH "The RiVLib ${basename} library")
      # Remove superfluous "debug" / "optimized" keywords from
      # RiVLib_LIBRARY_DIRS
      FOREACH(_rivlib_my_lib ${RiVLib_${basename}_LIBRARY})
        GET_FILENAME_COMPONENT(_rivlib_my_lib_path "${_rivlib_my_lib}" PATH)
        LIST(APPEND RiVLib_LIBRARY_DIRS ${_rivlib_my_lib_path})
      ENDFOREACH()
      LIST(REMOVE_DUPLICATES RiVLib_LIBRARY_DIRS)

      SET(RiVLib_LIBRARY_DIRS ${RiVLib_LIBRARY_DIRS} FILEPATH "RiVlib library directory")
      SET(RiVLib_${basename}_FOUND ON CACHE INTERNAL "Whether the RiVLib ${basename} component was found")
      SET(RiVLib_LIBRARIES ${RiVLib_LIBRARIES} ${RiVLib_${basename}_LIBRARY})
    ELSE(RiVLib_${basename}_LIBRARY)
        SET(RiVLib_FOUND FALSE) #FIXME: doesn't get propagated to caller
    ENDIF(RiVLib_${basename}_LIBRARY)
ENDFOREACH()

IF( RiVLib_CTRLIFC_LIBRARY)
    IF(MSVC)
        SET(RiVLib_CTRLIFC_RUNTIME_LIBRARIES ${RiVLib_DIR}/lib/ctrlifc-mt.dll)
    ENDIF(MSVC)
    IF(MINGW)
        SET(RiVLib_CTRLIFC_RUNTIME_LIBRARIES ${RiVLib_DIR}/lib/libctrlifc-mt.dll)
    ENDIF(MINGW)
    IF(UNIX)
        SET(RiVLib_CTRLIFC_RUNTIME_LIBRARIES ${RiVLib_DIR}/lib/libctrlifc-mt.so)
    ENDIF(UNIX)
ENDIF()

IF( RiVLib_SCANIFC_LIBRARY)
    IF(MSVC)
        SET(RiVLib_SCANIFC_RUNTIME_LIBRARIES ${RiVLib_DIR}/lib/scanifc-mt.dll)
    ENDIF(MSVC)
    IF(MINGW)
        SET(RiVLib_SCANIFC_RUNTIME_LIBRARIES ${RiVLib_DIR}/lib/libscanifc-mt.dll)
    ENDIF(MINGW)
    IF(UNIX)
        SET(RiVLib_SCANIFC_RUNTIME_LIBRARIES ${RiVLib_DIR}/lib/libscanifc-mt.so)
    ENDIF(UNIX)
ENDIF()

IF (RiVLib_FOUND)
    SET(RiVLib_INCLUDE_DIRS ${RiVLib_DIR}/include FILEPATH "RiVlib include directory")
    SET(RiVLib_ROOT_DIR ${RiVlib_DIR})
ENDIF()


