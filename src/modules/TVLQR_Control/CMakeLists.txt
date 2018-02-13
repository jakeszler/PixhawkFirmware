px4_add_module(
    MODULE modules__TVLQR_Control
    MAIN TVLQR_control
    COMPILE_FLAGS -Os
    STACK_MAIN 1200
    SRCS
        TVLQR_Control.cpp
    DEPENDS
        platforms__common
    )
