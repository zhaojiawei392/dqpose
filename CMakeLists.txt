 ## 
 ##    This file is part of dqpose.
 ## 
 ##    dqpose is free software: you can redistribute it and/or modify 
 ##    it under the terms of the GNU General Public License as published 
 ##    by the Free Software Foundation, either version 3 of the License, 
 ##    or (at your option) any later version.
 ## 
 ##    dqpose is distributed in the hope that it will be useful, 
 ##    but WITHOUT ANY WARRANTY; without even the implied warranty of 
 ##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 ##    See the GNU General Public License for more details.
 ## 
 ##    You should have received a copy of the GNU General Public License
 ##    along with dqpose. If not, see <https://www.gnu.org/licenses/>.
 ##

 ##
 ##    Filename CmakeLists.txt
 ##	   Author Jiawei ZHAO
 ##	   Version 1.0
 ##	   Date 2024-2025
 ##

cmake_minimum_required(VERSION 3.25)

# set the project name and version
project(dqpose VERSION 1.0 LANGUAGES CXX)
set(CMAKE_DEBUG_POSTFIX d)

# Package metadata
set(PACKAGE_NAME "dqpose")
set(PACKAGE_VERSION "1.0.0")
set(PACKAGE_SO_VERSION "1.0")
set(PACKAGE_AUTHOR "Jiawei ZHAO")
set(PACKAGE_MAINTAINER "Jiawei ZHAO")
set(PACKAGE_URL "https://github.com/zhaojiawei392/DualQuaternion/tree/2024-1.0a1")

add_library(dqpose_compiler_flags INTERFACE)
target_compile_features(dqpose_compiler_flags INTERFACE cxx_std_17)

# add compiler warning flags just when building this project via
# the BUILD_INTERFACE genex
set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
target_compile_options(dqpose_compiler_flags INTERFACE
  "$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused>>"
  "$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
)

# control where the static and shared libraries are built so that on windows
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
set(dqpose_BUILD_EXAMPLES ON CACHE BOOL "Build examples for dqpose" FORCE)
set(dqpose_BUILD_TESTS ON CACHE BOOL "Build examples for dqpose" FORCE)

if(APPLE)
  set(CMAKE_INSTALL_RPATH "@executable_path/../lib")
elseif(UNIX)
  set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
endif()

add_subdirectory(examples)
add_subdirectory(tests)

# enable testing
enable_testing()

# does the application run
add_test(NAME Runs COMMAND example1)

# setup installer
include(InstallRequiredSystemLibraries)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/License.txt")
set(CPACK_PACKAGE_VERSION_MAJOR "${dqpose_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${dqpose_VERSION_MINOR}")
set(CPACK_GENERATOR "TGZ")
set(CPACK_SOURCE_GENERATOR "TGZ")
include(CPack)

include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/dqposeConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/dqposeConfig.cmake"
  INSTALL_DESTINATION "lib/cmake/dqpose"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
  )
# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/dqposeConfigVersion.cmake"
  VERSION "${dqpose_VERSION_MAJOR}.${dqpose_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the generated configuration files
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/dqposeConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/dqposeConfigVersion.cmake
  DESTINATION lib/cmake/dqpose
  )