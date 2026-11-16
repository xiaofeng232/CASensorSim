# SPDX-License-Identifier: Apache-2.0
#
# CASensorSim -- CA (crank angle) sensor signal simulator firmware.
#
# Zephyr app for the NUCLEO-F429ZI test-bench board (matches the hw/ KiCad
# project in the platform workspace). The bit-exact cas_core / cas_fault
# port of the casim golden model is the active task -- see HANDOFF.md #7;
# until then this app is a board bring-up skeleton (LED heartbeat).
#
# Build from the zephyrproject workspace (this repo is registered in
# manifest/west.yml as apps/cas-sim):
#
#   west build -b nucleo_f429zi apps/cas-sim
#   west flash -d apps/cas-sim/build

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cas_sensor_sim LANGUAGES C)

# include/ holds the public core headers; src/ the implementation.
target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
    src/main.c
    # Bit-exact casim port (virtual CA core), host-tested with
    # test/test_golden.c; timer + GPIO wiring is a later step.
    src/cas_core.c
)
