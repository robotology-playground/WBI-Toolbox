# Copyright (C) 2013 CoDyCo
# Author: Jorhabib Eljaik
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# MATLAB and YARP is searched before adding the different modules because they will be used by all of them
find_package(YARP        REQUIRED)

# Fail if YARP is not compiled as shared library
# see https://github.com/robotology/codyco-modules/issues/44
if(NOT ${YARP_IS_SHARED_LIBRARY})
     message(FATAL_ERROR "YARP was found, but it was compiled as static library. A shared library version of YARP is required.")
endif()

find_package(Matlab	 REQUIRED
                         MX_LIBRARY
                         ENG_LIBRARY)

message("Matlab root dir " ${Matlab_ROOT_DIR})
message("Matlab version " ${Matlab_VERSION_STRING})

if(Matlab_FOUND)
        message("Matlab libraries found: ${Matlab_LIBRARIES}")
        message("Matlab mex library: " ${Matlab_MEX_LIBRARY})
        message("Matlab mx  library: " ${Matlab_MX_LIBRARY})
        message("Matlab ENG library: " ${Matlab_ENG_LIBRARY})
else()
        message(FATAL_ERROR "Matlab not found")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

#### Settings for rpath
if(NOT MSVC)
    #add the option to enable RPATH
    option(WBITOOLBOX_ENABLE_RPATH "Enable RPATH installation" TRUE)
    mark_as_advanced(WBITOOLBOX_ENABLE_RPATH)
endif(NOT MSVC)

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS ${CMAKE_INSTALL_PREFIX}/bin
                          LIB_DIRS ${CMAKE_INSTALL_PREFIX}/mex ${CMAKE_INSTALL_PREFIX}/lib
                          DEPENDS WBITOOLBOX_ENABLE_RPATH
                          USE_LINK_PATH)

add_subdirectory(wbInterface)
add_subdirectory(minJerkGenerator)
add_subdirectory(yRead)
add_subdirectory(yarpRead)
add_subdirectory(yWrite)
add_subdirectory(ySynchronizer)
add_subdirectory(realTimeSynchronizer)

set(WBI-TOOLBOX_SHARE_DIR "${CMAKE_INSTALL_PREFIX}/share/WBI-Toolbox")
set(WBI-TOOLBOX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../")
message("WBI-TOOLBOX_ROOT_DIR: " ${WBI-TOOLBOX_ROOT_DIR})

##if you export the mdl / slx library, remeber to call this command in matlab before saving it
# set_param(gcs, 'EnableLBRepository','on');

# The following line is to properly configure the installation script of the toolbox
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../.startup_wbitoolbox.m.in ${CMAKE_CURRENT_SOURCE_DIR}/../startup_wbitoolbox.m)

# Install configuration files
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../startup_wbitoolbox.m DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/images         DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../share/tools DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/slblocks.m DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
#if MAJOR >= 2014 && MINOR >= b
if (${Matlab_VERSION_STRING} GREATER "8.3")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/WBCLibrary.slx DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
else()
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/WBCLibrary.mdl DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
endif()
