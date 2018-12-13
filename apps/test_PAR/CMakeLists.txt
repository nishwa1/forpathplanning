#  +---------------------------------------------------------------------------+
#  |                              			                                   |
#  |                                                                           |
#  |                                                                           |
#  | Copyright (c) 2019, - All rights reserved.                                |
#  | Authors:                                                                  |
#  | Released under ___ License.                                               |
#  +---------------------------------------------------------------------------+ 

project(Test_PAS_rot)

set (APP_SOURCES_DIR ${PROJECT_SOURCE_DIR} )
FILE(GLOB APP_SOURCES "${APP_SOURCES_DIR}/*.cpp")  

#  including external libraries
include_directories(${PAR_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS date_time filesystem iostreams )

include_directories(${Boost_INCLUDE_DIR})
message (STATUS Boost_LIBRARIES  :::: ${Boost_LIBRARIES} )
message (STATUS PAR_INCLUDE_DIR  :::: ${PAR_INCLUDE_DIR} )

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()
	
add_executable (${PROJECT_NAME} ${APP_SOURCES})
target_link_libraries (${PROJECT_NAME} ${Boost_LIBRARIES} PAS )

# allows folders for MSVC
if (MSVC AND ENABLE_SOLUTION_FOLDERS) 
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Apps")
endif (MSVC AND ENABLE_SOLUTION_FOLDERS)
