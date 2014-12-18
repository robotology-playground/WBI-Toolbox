# Copyright (C) 2013 CoDyCo
# Author: Jorhabib Eljaik
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# MATLAB and YARP is searched before adding the different modules because they will be used by all of them
find_package(YARP        REQUIRED)
find_package(Matlab	 REQUIRED)

if(MATLAB_FOUND)
        message(STATUS "Matlab was found!! The configuration may continue")
        set(MATLAB_LIBRARIES
                ${MATLAB_MEX_LIBRARY}
                ${MATLAB_MX_LIBRARY}
                ${MATLAB_ENG_LIBRARY}
        )
else()
        message(FATAL_ERROR "Matlab not found")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

#### Settings for rpath
if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER "2.8.12")
    message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(NOT (CMAKE_VERSION VERSION_LESS 2.8.12))
    if(NOT MSVC)
        #add the option to disable RPATH
        set(WBITOOLBOX_DISABLE_RPATH FALSE CACHE BOOL "Disable RPATH installation")
        mark_as_advanced(WBITOOLBOX_DISABLE_RPATH)
    endif(NOT MSVC)

    #Configure RPATH
    set(CMAKE_MACOSX_RPATH 1) #enable RPATH on OSX. This also suppress warnings on CMake >= 3.0
    # when building, don't use the install RPATH already
    # (but later on when installing)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
    #build directory by default is built with RPATH
    set(CMAKE_SKIP_BUILD_RPATH  FALSE)

    #This is relative RPATH for libraries built in the same project
    #I assume that the directory is
    # - install_dir/something for binaries
    # - install_dir/lib for libraries
    file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_PREFIX}/bin" "${CMAKE_INSTALL_PREFIX}/lib")
    if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_INSTALL_RPATH "@loader_path/${_rel_path}")
    else()
        set(CMAKE_INSTALL_RPATH "\$ORIGIN/${_rel_path}")
    endif()
    
    file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_PREFIX}/bin" "${CMAKE_INSTALL_PREFIX}/mex")
    if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_INSTALL_RPATH "@loader_path/${_rel_path}")
    else()
        set(CMAKE_INSTALL_RPATH "\$ORIGIN/${_rel_path}")
    endif()

    # the RPATH to be used when installing, but only if it's not a system directory (copied form CMake File. To be tested)
    list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
    if("${isSystemDir}" STREQUAL "-1")
       set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    endif("${isSystemDir}" STREQUAL "-1")

    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the install RPATH
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE) #very important!

    if(WBITOOLBOX_DISABLE_RPATH)
        #what to do? disable RPATH altogether or just revert to the default CMake configuration?
        #I revert to default
        unset(CMAKE_INSTALL_RPATH) #remove install rpath
        set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
    endif()
endif()
#####end RPATH

add_subdirectory(wbInterface)
add_subdirectory(minJerkGenerator)
add_subdirectory(yRead)
add_subdirectory(yWrite)
add_subdirectory(softRealTime)
add_subdirectory(ySynchronizer)

set(WBI-TOOLBOX_SHARE_DIR "${CMAKE_INSTALL_PREFIX}/share/WBI-Toolbox")
set(WBI-TOOLBOX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../")
message("WBI-TOOLBOX_ROOT_DIR: " ${WBI-TOOLBOX_ROOT_DIR})

# The following line is to properly configure the installation script of the toolbox
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../.startup_wbitoolbox.m.in ${CMAKE_CURRENT_SOURCE_DIR}/../startup_wbitoolbox.m)

# Install configuration files
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../startup_wbitoolbox.m DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/images         DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../share/tools DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/slblocks.m DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/WBCLibrary.mdl DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/WBCLibrary.slx DESTINATION ${WBI-TOOLBOX_SHARE_DIR})
