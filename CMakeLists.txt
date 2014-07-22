cmake_minimum_required(VERSION 2.8)

option(CODYCO_USES_URDFDOM "Compile WBI-Toolbox with URDF support" TRUE)
option(CODYCO_ICUBWBI_USE_EXTERNAL_TORQUE_CONTROL "Compile yarpwholebodyinterface to use JTC external torque control" FALSE)

if(CODYCO_USES_URDFDOM)
    add_definitions(-DCODYCO_USES_URDFDOM)
endif()

if (CODYCO_ICUBWBI_USE_EXTERNAL_TORQUE_CONTROL)
    add_definitions(-DWBI_ICUB_COMPILE_PARAM_HELP)
endif()

#On new versions of Clang, MATLAB requires C++11.
#I enable it on all Clangs
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
        if (${CMAKE_GENERATOR} MATCHES "Xcode")
            #this should set explictly the option in xcode. Unfortunately it does not work.
            set(XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "C++11")
        endif(${CMAKE_GENERATOR} MATCHES "Xcode")
    endif(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_subdirectory(libraries)

# The following line is to properly configure the installation script of the toolbox
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/startup_wbitoolbox.m.in ${CMAKE_CURRENT_SOURCE_DIR}/startup_wbitoolbox.m)
