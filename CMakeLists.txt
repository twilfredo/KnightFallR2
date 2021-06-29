# SPDX-License-Identifier: Apache-2.0
#Setting to exclusively build for the Particle Argon
set(BOARD particle_boron)
set(BOARD_FLASH_RUNNER jlink)
set(CONF_FILE general_config.conf uart_shell_config.conf usb_config.conf)

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uqdrifter)

#Add include_directories for libraries, path starts from this files location.
include_directories(inc/
                    lib/saraR4_modem/
                    lib/sam_m8q_gps/
                    #lib/sensors_custom/
                    lib/tsd10/
                    lib/sensor_ctrl/
                    )

target_sources(app PRIVATE
                        src/main.c
                        lib/saraR4_modem/sara_r4.c      #LTE-M1 Driver
                        lib/saraR4_modem/dbg_led.c      #Debug Led Thread
                        lib/sam_m8q_gps/sam_m8q.c       #GPS Module
                        lib/tsd10/tsd10_adc.c   
                        lib/sensors_custom/adxl343.c    #Axlero
                        lib/sensors_custom/mcp9808.c    #Temp Sens
                        #lib/sensors_custom/sensors.c    #Sensor Integration Driver <-> HAL    
                        lib/sensor_ctrl/sensor_pwr.c    #Controls the PWR mosFETs for sensor power
                        lib/sensor_ctrl/sensor_ctrl.c   #Sensor HAL (Abstraction driver for controlling sensors)
             )
