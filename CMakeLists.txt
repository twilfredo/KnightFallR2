# SPDX-License-Identifier: Apache-2.0
#Setting to exclusively build for the Particle Argon
set(BOARD particle_boron)
set(BOARD_FLASH_RUNNER jlink)
set(CONF_FILE general_config.conf usb_config.conf uart_shell_config.conf)
set(DTC_OVERLAY_FILE dts/spi3.dtsi)

cmake_minimum_required(VERSION 3.20.4)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uqdrifter)

#Add include_directories for libraries, path starts from this files location.
include_directories(inc/
                    lib/saraR4_modem/
                    lib/sam_m8q_gps/
                    lib/sensor_ctrl/
                    lib/mcp3008_adc/
                    lib/sys_pwr/
                    lib/sys_led/
                    lib/jsmn/
                    )

target_sources(app PRIVATE
                        src/main.c
                        lib/mcp3008_adc/mcp3008.c
                        lib/saraR4_modem/sara_r4.c      #LTE-M1 Driver
                        lib/sys_led//dbg_led.c          #Debug Led Thread
                        lib/sam_m8q_gps/sam_m8q.c       #GPS Module  
                        lib/sensor_ctrl/sensor_pwr.c    #Controls the PWR mosFETs for sensor power
                        lib/sensor_ctrl/sensor_ctrl.c   #Sensor HAL (Abstraction driver for controlling sensors)
                        lib/sys_pwr/sys_pwr.c           #Boron Pwr control IC
             )
