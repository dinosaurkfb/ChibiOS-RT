# List of all the LPC13xx platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/platforms/LPC17xx/hal_lld.c \
			  ${CHIBIOS}/os/hal/platforms/LPC17xx/gpt_lld.c \
              ${CHIBIOS}/os/hal/platforms/LPC17xx/pal_lld.c \
              ${CHIBIOS}/os/hal/platforms/LPC17xx/serial_lld.c \
              ${CHIBIOS}/os/hal/platforms/LPC17xx/ext_lld.c \
              ${CHIBIOS}/os/hal/platforms/LPC17xx/ext_lld_isr.c \
              ${CHIBIOS}/os/hal/platforms/LPC17xx/system_LPC177x_8x.c \
              ${CHIBIOS}/os/hal/platforms/LPC17xx/lpc177x_8x_gpio.c \
              ${CHIBIOS}/os/hal/platforms/LPC17xx/lpc177x_8x_pinsel.c \
              ${CHIBIOS}/os/hal/platforms/LPC17xx/rtc_lld.c
             
         

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/platforms/LPC17xx
