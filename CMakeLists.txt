set(MRT_PKG_VERSION 4.0.0)
# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 3.5.1)
project(lanelet_rviz_plugin_ros)

###################
## Find packages ##
###################
find_package(mrt_cmake_modules REQUIRED)
include(UseMrtStdCompilerFlags)
include(GatherDeps)

# You can add a custom.cmake in order to add special handling for this package. E.g. you can do:
# list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
# To remove libs which cannot be found automatically. You can also "find_package" other, custom dependencies there.
# You can also set PROJECT_INSTALL_FILES to install files that are not installed by default.
# For packages that generate ROS messages, you can do e.g. the following to ensure messages are found:
# set(PROJECT_MESSAGE_DEPENDS std_msgs geometry_msgs)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
endif()

find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})

mrt_parse_package_xml()

########################
## Add python modules ##
########################
# This adds a python module if located under src/{PROJECT_NAME)
mrt_python_module_setup()

mrt_glob_files(PROJECT_PYTHON_SOURCE_FILES_SRC "python_api/*.cpp")
if (PROJECT_PYTHON_SOURCE_FILES_SRC)
    # Add a cpp-python api library. Make sure there are no name collisions with python modules in this project
    mrt_add_python_api( ${PROJECT_NAME}
        FILES ${PROJECT_PYTHON_SOURCE_FILES_SRC}
        )
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

# Add message, service and action files
mrt_add_message_files(msg)
mrt_add_service_files(srv)
mrt_add_action_files(action)

# Generate added messages and services with any dependencies listed here
if (ROS_GENERATE_MESSAGES)
    generate_messages(
        DEPENDENCIES ${PROJECT_MESSAGE_DEPENDS}
        )
endif()

# Generate dynamic reconfigure options
mrt_glob_files(PARAMS_FILES "cfg/*.params" "cfg/*.cfg" "cfg/*.mrtcfg" "cfg/*.rosif")
if (PARAMS_FILES)
    generate_ros_parameter_files(${PARAMS_FILES})
    generate_ros_interface_files(${PARAMS_FILES})
endif()

############################
## Read source code files ##
############################
mrt_glob_files_recurse(PROJECT_HEADER_FILES_INC "include/*.h" "include/*.hpp" "include/*.cuh")
mrt_glob_files(PROJECT_SOURCE_FILES_INC "src/*.h" "src/*.hpp" "src/*.cuh")
mrt_glob_files(PROJECT_SOURCE_FILES_SRC "src/*.cpp" "src/*.cu")

###########
## Build ##
###########
# Declare a cpp library
mrt_add_library(${PROJECT_NAME}
    INCLUDES ${PROJECT_HEADER_FILES_INC} ${PROJECT_SOURCE_FILES_INC}
    SOURCES ${PROJECT_SOURCE_FILES_SRC}
    )

#############
## Install ##
#############
# Install all targets, headers by default and scripts and other files if specified (folders or files).
# This command also exports libraries and config files for dependent packages and this supersedes catkin_package.
mrt_install(PROGRAMS scripts FILES launch rviz maps res data nodelet_plugins.xml plugin_description.xml ${PROJECT_INSTALL_FILES})

#############
## Testing ##
#############
# Add test targets for cpp and python tests
if (CATKIN_ENABLE_TESTING)
    mrt_add_ros_tests(test)
    mrt_add_tests(test)
    mrt_add_nosetests(test)
endif()
