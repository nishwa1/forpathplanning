#  +---------------------------------------------------------------------------+
#  |                              			                                   |
#  |                                                                           |
#  |                                                                           |
#  | Copyright (c) 2019, - All rights reserved.                                |
#  | Authors:                                                                  |
#  | Released under ___ License.                                               |
#  +---------------------------------------------------------------------------+ 

project(Test_PAS)

#  including external libraries
include_directories(${PAS_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS date_time filesystem iostreams graph)

include_directories(${Boost_INCLUDE_DIR})
message (STATUS TEST Boost_LIBRARIES  :::: ${Boost_LIBRARIES} )
message (STATUS TEST PAS  :::: ${PAS} )

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()
	
add_executable (${PROJECT_NAME} pushSwapTest.cpp)
target_link_libraries (${PROJECT_NAME} ${Boost_LIBRARIES} PAS )

# allows folders for MSVC
if (MSVC AND ENABLE_SOLUTION_FOLDERS) 
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Apps")
endif (MSVC AND ENABLE_SOLUTION_FOLDERS)
