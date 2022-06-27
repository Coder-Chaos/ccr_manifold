# tell cmake where to look for *.h files
include_directories(
    ${PROJECT_SOURCE_DIR}/3rdparty/do3think/include
    ${PROJECT_SOURCE_DIR}/src/hw_accelerator/resize_accel_v1_0/src 
    ${PROJECT_SOURCE_DIR}/utility
    ${PROJECT_SOURCE_DIR}/common
    ${PROJECT_SOURCE_DIR}/common/udmabuf
    ${PROJECT_SOURCE_DIR}/common/net
    ${PROJECT_SOURCE_DIR}/common/log
    ${PROJECT_SOURCE_DIR}/src/camera/ind_cam/do3think
    ${PROJECT_SOURCE_DIR}/src/camera/uvc
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

# set do3think source files
file(GLOB vision_src "ind_cam/do3think/*.cc"
                        "${PROJECT_SOURCE_DIR}/src/camera/uvc/*.cc"
                        "${PROJECT_SOURCE_DIR}/src/hw_accelerator/resize_accel_v1_0/src/*.c"
                        "${PROJECT_SOURCE_DIR}/common/udmabuf/*.cc"
                        "${PROJECT_SOURCE_DIR}/src/camera/ind_cam/do3think/*.cc"
                        "*.cc")

# do3think shared library
set(DO3THINK_LIB_DIR ${PROJECT_SOURCE_DIR}/3rdparty/do3think/lib)
file(GLOB DO3THINK_LIBS "${DO3THINK_LIB_DIR}/*.so")

# do3think executable
set(exe_bin_vision vision_capture)
add_executable(${exe_bin_vision} ${vision_src})

# link library
target_link_libraries(${exe_bin_vision} ${DO3THINK_LIBS} -lpthread -latomic common)