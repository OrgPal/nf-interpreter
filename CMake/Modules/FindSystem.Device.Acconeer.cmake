#
# Copyright (c) .NET Foundation and Contributors
# See LICENSE file in the project root for full license information.
#

# native code directory
set(BASE_PATH_FOR_THIS_MODULE ${BASE_PATH_FOR_CLASS_LIBRARIES_MODULES}/System.Device.Acconeer)


# set include directories
list(APPEND System.Device.Acconeer_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src/CLR/Core)
list(APPEND System.Device.Acconeer_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src/CLR/Include)
list(APPEND System.Device.Acconeer_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src/HAL/Include)
list(APPEND System.Device.Acconeer_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src/PAL/Include)
list(APPEND System.Device.Acconeer_INCLUDE_DIRS ${BASE_PATH_FOR_THIS_MODULE})
list(APPEND System.Device.Acconeer_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src/System.Device.Acconeer)

# source files
set(System.Device.Acconeer_SRCS

    sys_dev_acconeer.cpp


    sys_dev_acconeer_System_Device_Acconeer_Sensor.cpp

)

foreach(SRC_FILE ${System.Device.Acconeer_SRCS})

    set(System.Device.Acconeer_SRC_FILE SRC_FILE-NOTFOUND)

    find_file(System.Device.Acconeer_SRC_FILE ${SRC_FILE}
        PATHS
	        ${BASE_PATH_FOR_THIS_MODULE}
	        ${TARGET_BASE_LOCATION}
            ${PROJECT_SOURCE_DIR}/src/System.Device.Acconeer

	    CMAKE_FIND_ROOT_PATH_BOTH
    )

    if (BUILD_VERBOSE)
        message("${SRC_FILE} >> ${System.Device.Acconeer_SRC_FILE}")
    endif()

    list(APPEND System.Device.Acconeer_SOURCES ${System.Device.Acconeer_SRC_FILE})

endforeach()

include(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(System.Device.Acconeer DEFAULT_MSG System.Device.Acconeer_INCLUDE_DIRS System.Device.Acconeer_SOURCES)
