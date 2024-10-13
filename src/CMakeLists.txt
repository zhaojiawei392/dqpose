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

 message(STATUS "BUILD_SHARED_LIBS is set to ${BUILD_SHARED_LIBS}")
 add_library(dqpose INTERFACE)
 
 set_property(TARGET dqpose PROPERTY VERSION ${PACKAGE_VERSION})
 set_property(TARGET dqpose PROPERTY SOVERSION ${PACKAGE_SO_VERSION})
 
 target_include_directories(dqpose
                            PUBLIC
                             $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/dqpose>
                             $<INSTALL_INTERFACE:include/dqpose>
                            )
 # state that SqrtLibrary need PIC when the default is shared libraries
 set_target_properties(dqpose PROPERTIES
                     POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
                     )
 
 target_link_libraries(dqpose PUBLIC dqpose_compiler_flags)
 
 set(installable_libs dqpose dqpose_compiler_flags)
 install(TARGETS ${installable_libs}
     EXPORT dqposeTargets
     DESTINATION lib
 )
 install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/ 
     DESTINATION include 
     FILES_MATCHING PATTERN "*.hpp")
 
 
 # Install pybind
 # Find Python and Pybind11
 find_package(Python3 REQUIRED COMPONENTS Interpreter Development)  # Finds Python
 find_package(pybind11 REQUIRED)  # Finds Pybind11
 
 # Name of the Python module
 set(MODULE_NAME "pydqpose")
 
 # Add the library (the Python module)
 pybind11_add_module(${MODULE_NAME} pywarpper.cpp)
 
 # Installation target
 install(TARGETS ${MODULE_NAME}
         LIBRARY DESTINATION ${Python3_SITEARCH})  # Destination for the module
 