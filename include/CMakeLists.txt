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
 ##    Filename src/CmakeLists.txt
 ##	   Author Jiawei ZHAO
 ##	   Version 1.0
 ##	   Date 2024-2025
 ##

 add_library(dqpose INTERFACE)
 
 set_property(TARGET dqpose PROPERTY VERSION ${PACKAGE_VERSION})
 set_property(TARGET dqpose PROPERTY SOVERSION ${PACKAGE_SO_VERSION})
 
 target_include_directories(dqpose INTERFACE
                            INTERFACE
                            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
                            )

# Install the headers
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
    DESTINATION include 
    FILES_MATCHING PATTERN "*.hpp")
 
# Install the target and create the export set (dqposeConfig)
install(TARGETS dqpose
    EXPORT dqposeTargets
    DESTINATION lib/cmake/dqpose
)

# Generate dqposeConfig.cmake and dqposeConfigVersion.cmake
install(EXPORT dqposeTargets
    FILE dqposeConfig.cmake
    NAMESPACE dqpose::
    DESTINATION lib/cmake/dqpose
)

# Optionally create a version file (dqposeConfigVersion.cmake) for version checking
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/dqposeConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install the version file
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/dqposeConfigVersion.cmake"
    DESTINATION lib/cmake/dqpose
)