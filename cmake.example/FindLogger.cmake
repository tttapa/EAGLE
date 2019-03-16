find_path(LOGGER_INCLUDE_DIR
    NAMES 
        logger.h
    PATHS 
        $ENV{HOME}/PO-EAGLE/Groups/ANC/Cleanup-Pieter/Code-Generators/
        # Change this path
    PATH_SUFFIXES
        Logger/Output/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Logger
    REQUIRED_VARS LOGGER_INCLUDE_DIR
)

get_filename_component(LOGGER_LIBRARIES 
    "${LOGGER_INCLUDE_DIR}/../../x86_64/lib/liblogger.a" 
    ABSOLUTE
)
message("Logger include ${LOGGER_INCLUDE_DIR}")
message("Logger libraries ${LOGGER_LIBRARIES}")

add_custom_target(make-logger
                  make
                  WORKING_DIRECTORY "${LOGGER_INCLUDE_DIR}/../../")

if(NOT TARGET logger)
    add_library(logger INTERFACE IMPORTED)
    add_dependencies(logger make-logger)
    set_target_properties(logger PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${LOGGER_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${LOGGER_LIBRARIES}"
    )
endif()