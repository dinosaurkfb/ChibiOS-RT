# List of all the ChibiOS/RT devices lib files, there is no need to remove the files
# from this list, you can disable parts of the devices lib by editing board.h.
DEVICESLIBSRC = ${CHIBIOS}/os/various/devices_lib/accel/lis302dl.c \
                ${CHIBIOS}/os/various/devices_lib/eeprom/at24c0x.c \
                ${CHIBIOS}/os/various/devices_lib/eeprom/mc24lc0x.c \
                ${CHIBIOS}/os/various/devices_lib/spi_flash/SST25VF016B.c \
                ${CHIBIOS}/os/various/devices_lib/spi_flash/FlashDriver.c

# Required include directories
DEVICESLIBINC = ${CHIBIOS}/os/various/devices_lib/ \
                ${CHIBIOS}/os/various/devices_lib/accel/ \
                ${CHIBIOS}/os/various/devices_lib/eeprom/ \
                ${CHIBIOS}/os/various/devices_lib/spi_flash/ 
