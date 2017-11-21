#pkg_version=2.0.3
# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(lanelet_rviz_plugin_ros)

###################
## find packages ##
###################
find_package(mrt_cmake_modules REQUIRED)
include(UseMrtStdCompilerFlags)
include(UseMrtAutoTarget)

include(GatherDeps)
#remove libs, which cannot be find automatically
#list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})

set(CMAKE_AUTOMOC ON)
#manually resolve removed dependend packages
#find_package(...)
################################################
## Declare ROS messages, services and actions ##
################################################

# Add message, service and action files
glob_ros_files(add_message_files msg)
glob_ros_files(add_service_files srv)
glob_ros_files(add_action_files action)

# Generate added messages and services with any dependencies listed here
if (ROS_GENERATE_MESSAGES)
	generate_messages(
	  DEPENDENCIES
	  #add dependencies here
	)
endif()

# Generate dynamic reconfigure options
file(GLOB CFG_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "cfg/*.cfg")
if (CFG_FILES)
	generate_dynamic_reconfigure_options(${CFG_FILES})
endif()


############################
## read source code files ##
############################
file(GLOB_RECURSE PROJECT_HEADER_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "include/*.h")
file(GLOB PROJECT_SOURCE_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.h")
file(GLOB PROJECT_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.cpp")

if (PROJECT_SOURCE_FILES_SRC)
	set(LIBRARY_NAME ${PROJECT_NAME})
endif()
 
###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${mrt_EXPORT_INCLUDE_DIRS}
  LIBRARIES ${LIBRARY_NAME} ${mrt_EXPORT_LIBRARIES}
  CATKIN_DEPENDS ${catkin_EXPORT_DEPENDS}
)

###########
## Build ##
###########
# Add include and library directories
include_directories(
  include/${LIBRARY_NAME}
  ${mrt_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${mrt_LIBRARY_DIRS}
)

if (PROJECT_SOURCE_FILES_SRC)
	# Declare a cpp library
	add_library(${LIBRARY_NAME}
	  ${PROJECT_HEADER_FILES_INC}
	  ${PROJECT_SOURCE_FILES_INC}
	  ${PROJECT_SOURCE_FILES_SRC}
	)
	
	# Add cmake target dependencies of the executable/library
	# as an example, message headers may need to be generated before nodes
	add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
	
	# Specify libraries to link a library or executable target against
	target_link_libraries(${PROJECT_NAME}
	  ${catkin_LIBRARIES}
	  ${mrt_LIBRARIES}
	)
endif()


#############
## Install ##
#############
if (TARGET ${PROJECT_NAME})
	# Mark library for installation
	install(TARGETS ${PROJECT_NAME}
	  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endif()

# Mark c++ header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#install(FILES
#  res/test.png
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#############
## Testing ##
#############
# Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	file(GLOB PROJECT_TEST_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test/*.cpp")
	foreach(PROJECT_TEST_FILE_SRC ${PROJECT_TEST_FILES_SRC})
		get_filename_component(PROJECT_TEST_NAME ${PROJECT_TEST_FILE_SRC} NAME_WE)
		
		catkin_add_gtest(${PROJECT_TEST_NAME}-test ${PROJECT_TEST_FILE_SRC})
		target_link_libraries(${PROJECT_TEST_NAME}-test ${LIBRARY_NAME} ${catkin_LIBRARIES} ${mrt_LIBRARIES} gtest_main)
	endforeach()
endif()

