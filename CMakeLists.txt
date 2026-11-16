# SPDX-License-Identifier: Apache-2.0
#
# CASensorSim -- CA (crank angle) sensor signal simulator firmware.
#
# Zephyr app for the FK407M2-ZGT6 (STM32F407ZGT6) bench board. The
# bit-exact cas_core / cas_fault port of the casim golden model lives in
# cas/ (include/ + src/, plain library, host-testable with test/).
#
# The board is out-of-tree (boards/others/fk407m2_zgt6 in the platform
# workspace), so the workspace root is added to BOARD_ROOT.
#
# Build from the zephyrproject workspace (this repo is registered in
# manifest/west.yml as apps/cas-sim):
#
#   west build -p always -b fk407m2_zgt6 apps/cas-sim
#   west flash -d apps/cas-sim/build

cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board root (boards/others/fk407m2_zgt6).
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cas_sensor_sim LANGUAGES C)

add_subdirectory(cas)

target_sources(app PRIVATE
    ./main.c
)
