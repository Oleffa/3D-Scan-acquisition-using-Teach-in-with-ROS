#----------------------------------------------------------------
# Generated CMake target import file for configuration "".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "odo" for configuration ""
set_property(TARGET odo APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(odo PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libodo.so"
  IMPORTED_SONAME_NOCONFIG "libodo.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS odo )
list(APPEND _IMPORT_CHECK_FILES_FOR_odo "${_IMPORT_PREFIX}/lib/libodo.so" )

# Import target "VMCLIB" for configuration ""
set_property(TARGET VMCLIB APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(VMCLIB PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libVMCLIB.so"
  IMPORTED_SONAME_NOCONFIG "libVMCLIB.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS VMCLIB )
list(APPEND _IMPORT_CHECK_FILES_FOR_VMCLIB "${_IMPORT_PREFIX}/lib/libVMCLIB.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
