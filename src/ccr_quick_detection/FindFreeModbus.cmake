MESSAGE(STATUS "Using bundled FindFreeModbus.cmake...")
  FIND_PATH(
  FREEMODBUS_INCLUDE_DIR
  mb.h 
  ~/data/Chaos/CCR/Ref/Modbus/freemodbus-v1.6/demo/LINUXTCP/modbus
  )

FIND_LIBRARY(
  FREEMODBUS_LIBRARIES NAMES  FreeModbus
  PATHS ~/data/Chaos/CCR/Ref/Modbus/freemodbus-v1.6/demo/LINUXTCP
  )



