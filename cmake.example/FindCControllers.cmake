find_path(CCONTROLLERS_INCLUDE_DIR
    NAMES 
        attitude-controller.h
        altitude-controller.h
    PATHS 
        $ENV{HOME}/PO-EAGLE/Groups/ANC/Cleanup-Pieter/Code-Generators/
        # Change this path
    PATH_SUFFIXES
        Controllers/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CControllers
    REQUIRED_VARS CCONTROLLERS_INCLUDE_DIR
)

get_filename_component(CCONTROLLERS_LIBRARIES 
    "${CCONTROLLERS_INCLUDE_DIR}/../x86_64/lib/libcontrollers.a" 
    ABSOLUTE
)
get_filename_component(CCONTROLLERS_PARAMS_AND_MATRICES
    "${CCONTROLLERS_INCLUDE_DIR}/../Output/ParamsAndMatrices"
    ABSOLUTE
)
message("CControllers include ${CCONTROLLERS_INCLUDE_DIR}")
message("CControllers libraries ${CCONTROLLERS_LIBRARIES}")
message("CControllers params ${CCONTROLLERS_PARAMS_AND_MATRICES}")

add_custom_target(make-c-controllers 
                  make
                  WORKING_DIRECTORY "${CCONTROLLERS_INCLUDE_DIR}/../")

if(NOT TARGET CControllers)
    add_library(CControllers INTERFACE IMPORTED)
    add_dependencies(CControllers make-c-controllers)
    set_target_properties(CControllers PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CCONTROLLERS_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${CCONTROLLERS_LIBRARIES}"
        INTERFACE_COMPILE_DEFINITIONS "PARAMS_AND_MATRICES_PATH=\"${CCONTROLLERS_PARAMS_AND_MATRICES}\""
    )
endif()