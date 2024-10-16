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
 
 target_include_directories(dqpose
                            PUBLIC
                            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
                            )

 install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
    DESTINATION include 
    FILES_MATCHING PATTERN "*.hpp")
 