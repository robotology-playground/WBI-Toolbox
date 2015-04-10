# Copyright (C) 2013 CoDyCo
# Author: Francesco Romano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.11)
PROJECT(yarpRead)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

set(SRC src/yarpRead.cpp)

source_group("Source Files" FILES ${SRC})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories("${Matlab_ROOT_DIR}/simulink/include")
include_directories("${Matlab_ROOT_DIR}/extern/include")
include_directories(${Matlab_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})

#Specify preprocessor flag for MATLAB MEX FILES Compilation.
add_definitions(-DMATLAB_MEX_FILE)

#Adding files used for the generation of the dynamic library
if (CMAKE_VERSION GREATER 3.0.0)
matlab_add_mex(
         NAME ${PROJECT_NAME}
         SRC ${SRC}
         LINK_TO ${YARP_LIBRARIES}
     )
else()
    #Adding files used for the generation of the dynamic library
    add_library(${PROJECT_NAME} SHARED ${SRC})

    set(SFUNCTION_SUFFIX ${Matlab_MEX_EXTENSION})
    #Removing default dynamic library extensions (.dll for Windows, .so for UNIX) and prefixes (lib) otherwise automatically prepended to the mex file.
    set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX .${SFUNCTION_SUFFIX})
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

    # This seems to be the most reliable method to date to know installed Matlab arch 
    STRING(REGEX MATCH ..$ MATLAB_ARCH Matlab_MEX_EXTENSION 64)

    #In the following specific flags will be set to ensure compliance with the default settings of Matlab's mex compiler
    if(WIN32)
            if(NOT "${MATLAB_ARCH}" STREQUAL "64")
                    message("Flags have been passed to compiler for Matlab 32bits on a Windows computer")
            elseif()
                    message("Flags have been passed to compiler for Matlab 64bits on a Windows computer")
            endif()
                    if(MSVC)
                            #Setting Compiler Flags
                            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Z7 /nologo /W3 /WX- /O2 /Oy- /D_REENTRANT /D_CRT_SECURE_NO_DEPRECATE /D_SCL_SECURE_NO_DEPRECATE /D_SECURE_SCL=0 /DyWrite_EXPORTS /Gm- /EHs /Zp8 /GS /fp:precise /Zc:wchar_t /Zc:forScope /GR /Gd /TP /showIncludes")
                            #Setting Linker Flags
                            set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /INCREMENTAL:NO /NOLOGO /MANIFEST /debug /MAP.mexw64.map /MACHINE:X64 /EXPORT:mexFunction")
                            if(NOT  "${MATLAB_ARCH}" STREQUAL "64")
                                    #Changing some flags according to system/MATLAB version (32, 64 bits)
                                    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /MAP.mexw32.map /MACHINE:X32")
                            endif()
                    elseif(MSVC)
                            message("You are not compiling using Microsoft Visual Studio, ERRORS might occur")
                    endif(MSVC)
    elseif(WIN32)
            message("No particular flags have been passed to compiler for Matlab 64bits on a UNIX computer")
    endif(WIN32)

    # Linking Libraries
    target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${Matlab_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION mex)
