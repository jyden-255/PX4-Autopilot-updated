px4_add_module(
    MODULE modules__crosswind_compensator
    MAIN crosswind_compensator
    SRCS
        CrosswindCompensator.cpp
    DEPENDS
        mathlib
        geo
    MODULE_CONFIG
        module.yaml
)
