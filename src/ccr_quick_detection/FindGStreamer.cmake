MESSAGE(STATUS "Using bundled Findgstreamer.cmake...")
  FIND_PATH(
  GSTREAMER_INCLUDE_DIR
  gst.h 
  /usr/include/gstreamer-1.0/gst
  /usr/local/include/gstreamer-1.0/gst
  /usr/include/gstreamer-1.0
  /usr/local/include/gstreamer-1.0
  /usr/include/glib-2.0
  /usr/lib/aarch64-linux-gnu/glib-2.0/include
  )

FIND_LIBRARY(
  GSTREAMER_LIBRARIES NAMES  gstreamer
  PATHS /usr/local/lib/gstreamer-1.0
  )



