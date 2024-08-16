#
# Copyright (c) .NET Foundation and Contributors
# See LICENSE file in the project root for full license information.
#

include(FetchContent)

function(nf_add_acconner_library)

    FetchContent_Declare(
        acconeer
        SOURCE_DIR ${ACCONEER_SOURCE}
    )

    # Check if population has already been performed
    FetchContent_GetProperties(acconeer)
    if(NOT acconeer_POPULATED)
        # Fetch the content using previously declared details
        FetchContent_MakeAvailable(acconeer)

        # add extra source files
        set(acconeer_SOURCES

            ${BASE_PATH_FOR_CLASS_LIBRARIES_MODULES}/platform_acc_hal_integration.c
        )

        # make var global
        set(acconeer_SOURCES ${acconeer_SOURCES} CACHE INTERNAL "make global")

    endif()

endfunction()
