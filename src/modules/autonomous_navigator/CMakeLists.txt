px4_add_module(
    MODULE modules__autonomous_navigator
    MAIN autonomous_navigator
    SRCS
        AutonomousNavigator.cpp
    DEPENDS
        mathlib
        geo
    MODULE_CONFIG
        module.yaml
)
