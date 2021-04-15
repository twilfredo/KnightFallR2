# SPDX-License-Identifier: Apache-2.0
#Setting to exclusively build for the Particle Argon
set(BOARD particle_boron)
set(BOARD_FLASH_RUNNER jlink)
set(DTC_OVERLAY_FILE boards/mcp9808.overlay)
set(CONF_FILE prj.conf uart_shell.conf modem.conf sensors.conf)

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blinky)

target_sources(app PRIVATE src/main.c)
