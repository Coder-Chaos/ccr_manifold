# freemodbus sources
set(freemodbus_sources
    LINUXTCP/port/portevent.c
    LINUXTCP/port/portother.c
    LINUXTCP/port/porttcp.c
    modbus/mb.c
    modbus/tcp/mbtcp.c
    modbus/functions/mbfunccoils.c
    modbus/functions/mbfuncdiag.c
    modbus/functions/mbfuncdisc.c
    modbus/functions/mbfuncholding.c
    modbus/functions/mbfuncinput.c
    modbus/functions/mbfuncother.c
    modbus/functions/mbutils.c
)

# add sources to lib
add_library(freemodbus_lib ${freemodbus_sources})

# add include
target_include_directories(freemodbus_lib 
                            PUBLIC LINUXTCP/port 
                            PUBLIC modbus/include
                            PUBLIC modbus/tcp
                            )

                           