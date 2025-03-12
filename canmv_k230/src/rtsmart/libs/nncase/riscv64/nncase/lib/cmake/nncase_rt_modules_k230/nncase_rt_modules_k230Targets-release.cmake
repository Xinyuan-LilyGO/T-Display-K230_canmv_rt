#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "functional_k230" for configuration "Release"
set_property(TARGET functional_k230 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(functional_k230 PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C;CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libfunctional_k230.a"
  )

list(APPEND _cmake_import_check_targets functional_k230 )
list(APPEND _cmake_import_check_files_for_functional_k230 "${_IMPORT_PREFIX}/lib/libfunctional_k230.a" )

# Import target "nncase_rt_modules_k230" for configuration "Release"
set_property(TARGET nncase_rt_modules_k230 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nncase_rt_modules_k230 PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C;CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libnncase.rt_modules.k230.a"
  )

list(APPEND _cmake_import_check_targets nncase_rt_modules_k230 )
list(APPEND _cmake_import_check_files_for_nncase_rt_modules_k230 "${_IMPORT_PREFIX}/lib/libnncase.rt_modules.k230.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
