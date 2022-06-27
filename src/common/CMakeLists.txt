include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/net
    ${CMAKE_CURRENT_SOURCE_DIR}/log
)

#lib sources
file(GLOB common_lib_src "*.cc" 
                        "net/*.cc"
                        "log/*.c"
                        )

                        
add_library(common STATIC ${common_lib_src})
