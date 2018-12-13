#  +---------------------------------------------------------------------------+
#  |                              			                                   |
#  |                                                                           |
#  |                                                                           |
#  | Copyright (c) 2019, - All rights reserved.                                |
#  | Authors:                                                                  |
#  | Released under ___ License.                                               |
#  +---------------------------------------------------------------------------+ 

# Note: This CMake file is not standalone, it defines the library 

project(PAR)   #define the project 

set (LIB_SOURCES_DIR ${PROJECT_SOURCE_DIR} )
set (LIB_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include)

#select all headers in the include dir
FILE(GLOB LIB_HEADERS "${LIB_HEADERS_DIR}/*.h")   
#select all sources in the include dir
FILE(GLOB LIB_SOURCES "${LIB_SOURCES_DIR}/src/*.cpp")  

message (STATUS "${PROJECT_NAME} MESSAGE:   ---   LIB_SOURCES    :: ${LIB_SOURCES}")
message (STATUS "${PROJECT_NAME} MESSAGE:   ---   LIB_HEADERS    :: ${LIB_HEADERS}")

#Allows other components to find this library
set (PAR_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include  CACHE PATH "Path to the headers of your lib") 
# necessary to allow the *.cpp to find headers 
include_directories(${PAR_INCLUDE_DIR})

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_definitions(-DPAS) 

find_package(Boost REQUIRED COMPONENTS date_time filesystem iostreams )

include_directories(${Boost_INCLUDE_DIR})
message (STATUS Boost_LIBRARIES  :::: ${Boost_LIBRARIES} )

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# add library 
add_library(${PROJECT_NAME}  SHARED ${LIB_SOURCES} ${LIB_HEADERS} )

target_link_libraries ( ${PROJECT_NAME}  ${Boost_LIBRARIES})

# set the postfix _d for debug libraries 
set_target_properties(${PROJECT_NAME}  PROPERTIES DEBUG_POSTFIX "_d")
set (PAR_INCLUDE_DIR ${lib_SOURCE_DIR}/include CACHE PATH "Path to the headers of the lib")

# allows folders for MSVC
if (MSVC AND ENABLE_SOLUTION_FOLDERS) 
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "libraries")
endif (MSVC AND ENABLE_SOLUTION_FOLDERS)