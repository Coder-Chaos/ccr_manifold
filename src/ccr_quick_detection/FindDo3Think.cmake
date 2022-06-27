MESSAGE(STATUS "Using bundled FindDo3Think.cmake...")
  FIND_PATH(
  DO3THINK_INCLUDE_DIR
  DVPCamera.h 
  ~/data/Chaos/CCR/Ref/DVP2-ARM64/Demo
  )

FIND_LIBRARY(
  DO3THINK_LIBRARIES NAMES  Demo_arm64
  PATHS ~/data/Chaos/CCR/Ref/DVP2-ARM64/
  )



