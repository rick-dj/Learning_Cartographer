#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cartographer_ros::cartographer_ros" for configuration ""
set_property(TARGET cartographer_ros::cartographer_ros APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(cartographer_ros::cartographer_ros PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcartographer_ros.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS cartographer_ros::cartographer_ros )
list(APPEND _IMPORT_CHECK_FILES_FOR_cartographer_ros::cartographer_ros "${_IMPORT_PREFIX}/lib/libcartographer_ros.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
