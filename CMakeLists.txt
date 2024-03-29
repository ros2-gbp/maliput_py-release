##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(maliput_py LANGUAGES C CXX VERSION 3.0.0)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(maliput REQUIRED)
find_package(pybind11 REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

message(STATUS "\n\n========= Project Configuration ========\n")

set(BUILD_SHARED_LIBS true)

include(${PROJECT_SOURCE_DIR}/cmake/DefaultCFlags.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/SanitizersConfig.cmake)

ament_python_install_package("maliput")

ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
)

##############################################################################
# Sources
##############################################################################
add_subdirectory(src)

##############################################################################
# Tests
##############################################################################

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  enable_testing()
  add_subdirectory(test)
  ament_flake8("--config" ${CMAKE_CURRENT_SOURCE_DIR}/.flake8)
  ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()

##############################################################################
# Docs
##############################################################################

if(BUILD_DOCS)
  message(STATUS "Doxygen generation - Enabled")
  find_package(ament_cmake_doxygen REQUIRED)
  ament_doxygen_generate(doxygen_maliput_py
    CONFIG_OVERLAY doc/Doxyfile.overlay.in
    DEPENDENCIES maliput
    TEST_ON_WARNS
  )
  add_definitions(-DBUILD_DOCS)
else()
  message(STATUS "Doxygen generation - Disabled")
endif()

##############################################################################
# Export
##############################################################################

ament_export_dependencies(ament_cmake)
ament_package()
