#  +---------------------------------------------------------------------------+
#  |                              			                                   |
#  |                                                                           |
#  |                                                                           |
#  | Copyright (c) 2019, - All rights reserved.                                |
#  | Authors:                                                                  |
#  | Released under ___ License.                                               |
#  +---------------------------------------------------------------------------+ 

project(DAT295_2018_MB1)   # DAT295_2018

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


#cmake tweaks
if(COMMAND cmake_policy) #return true if the COMMAND cmake_policy exist - this allows compatibility with older versions
	# We don't want to mix relative and absolute paths in linker lib lists.
	cmake_policy(SET CMP0003 NEW) # Required by CMake 2.7+
	
	# avoid link warnings - see http://www.cmake.org/cmake/help/v3.0/policy/CMP0015.html
	cmake_policy(SET CMP0015 NEW)
	
	if ("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 3.0)
         #Ignore COMPILE_DEFINITIONS_<Config> properties - see http://www.cmake.org/cmake/help/v3.0/policy/CMP0043.html
	cmake_policy(SET CMP0043 NEW) 
	endif ("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 3.0 )

	# Allow using the LOCATION target property.
	if(NOT "${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
		cmake_policy(SET CMP0026 OLD)
	endif(NOT "${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
endif(COMMAND cmake_policy)

include(${PROJECT_SOURCE_DIR}/CMakeModules/script_init_cmake.cmake REQUIRED)		# CMake configurations
include(${PROJECT_SOURCE_DIR}/CMakeModules/UtilsMacros.cmake REQUIRED)		        # CMake macros 

set (PR_SOURCE_DIR ${PROJECT_SOURCE_DIR})

option(ENABLE_verbose "Verbose cmake cofiguration " "ON")
if ( ENABLE_verbose )
	message(WARNING " Verbose configuration active ! You may get a lot of messages now ... " )
endif ( ENABLE_verbose )

# --------------------------------------
#  FIND all required libraries 
# --------------------------------------
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED)# COMPONENTS date_time filesystem iostreams )
include_directories(${Boost_INCLUDE_DIR})
message (STATUS Boost_LIBRARIES  :::: ${Boost_LIBRARIES} )



# --------------------------------------
#  Add options to build specific targets
# --------------------------------------
option(ENABLE_SOLUTION_FOLDERS "Enable folders for MSVC - ON by default if MSVC - OFF otherwise" ON)
if (MSVC)
  set (ENABLE_SOLUTION_FOLDERS 1)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
else (MSVC)
  set (ENABLE_SOLUTION_FOLDERS 0)
endif (MSVC) 

# add all libraries in the /libs folder
add_subdirectory(libs) 

# -----------  Pr_lib  --------------------
#include_directories("${Pr_lib_INCLUDE_DIR}")   #TODO for you

# -------------------- apps --------------------------------
OPTION(ENABLE_BUILD_apps "Option for building apps - ON by default" ON)
if (ENABLE_BUILD_apps)
	add_subdirectory(apps)
endif(ENABLE_BUILD_apps)


# --------------------------------------
#  settings for install and package --- not to do now
# --------------------------------------				
#if (MSVC)	
#include(${PROJECT_SOURCE_DIR}/CMakeModules/script_BuildInstall.cmake REQUIRED)		# CMake configurations
#include(${PROJECT_SOURCE_DIR}/CMakeModules/script_BuildPackage.cmake REQUIRED)		# CMake configurations
#endif(MSVC)						
