# SPDX-License-Identifier: Apache-2.0
#Setting to exclusively build for the Particle Argon
set(BOARD particle_boron)
set(BOARD_FLASH_RUNNER jlink)
set(DTC_OVERLAY_FILE boards/mcp9808.overlay)
set(CONF_FILE prj.conf rtt_console.conf)

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uqdrifter)

#Add include_directories for libraries, path starts from this files location.
include_directories(inc/
                    lib/saraR4_modem/
                    lib/sensors_custom/
                    )

target_sources(app PRIVATE
                        src/main.c
                        lib/saraR4_modem/sara_r4.c    
                        #lib/tsd10/tsd10_adc.c   
                        lib/sensors_custom/adxl343.c
                        lib/sensors_custom/mcp9808.c
                        lib/sensors_custom/sensors.c
                        )
