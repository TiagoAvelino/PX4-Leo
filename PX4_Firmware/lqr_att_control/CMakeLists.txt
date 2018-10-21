px4_add_module(
    MODULE modules__lqr_att_control
    MAIN lqr_att_control
    COMPILE_FLAGS
    STACK_MAIN 10000
    SRCS
        lqr_att_control.cpp
#	aux_func.cpp
	aux_func2.cpp
    DEPENDS
        platforms__common
    )

#file(GLOB MY_PUBLIC_HEADERS
#	dados_k_n.txt
#)
#set( MICROSD ${PX4_SOURCE_DIR}/fs/microsd )
#file(COPY ${MY_PUBLIC_HEADERS} DESTINATION ${MICROSD})
