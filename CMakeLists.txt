# SPDX-License-Identifier: Apache-2.0
#
# CASensorSim -- CA (crank angle) sensor signal simulator firmware.
#
# Zephyr app for the FK407M2-ZGT6 (STM32F407ZGT6) bench board. The
# bit-exact cas_core / cas_fault port of the casim golden model lives in
# cas/ (include/ + src/, plain library, host-testable with test/). The app
# layer (src/main.cpp) runs the 100 kHz TIM8 signal path and publishes the
# CasIntf telemetry/parameter interface over Fibre on the native USB bulk
# interface (Type-C connector; UART fallback on boards without a device
# controller); the interface autogen code is generated at build time from
# interface/cas_interface.yaml
# with the generator shipped in the Fibre module -- nothing generated is
# committed.
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

# Version machinery (odrive_study model, doc/design/mfcs-007.md): the app
# line derives from this repo's git tags (+ version.txt target), the
# platform line from the workspace repo, checked against
# platform-requires.txt at configure time. The init calls must run before
# find_package(Zephyr) so the generated version.conf reaches kconfig.
include(${CMAKE_CURRENT_LIST_DIR}/../../scripts/git_version.cmake)
git_version_init(${CMAKE_CURRENT_LIST_DIR})
include(${CMAKE_CURRENT_LIST_DIR}/../../scripts/platform_version.cmake)
platform_version_init(${CMAKE_CURRENT_LIST_DIR})

# Platform lib module: provides the boot banner (util/banner), the Fibre
# transport backends (services/fibre-transport) and the rest of the platform
# services -- all of those default to n, so only what prj.conf enables
# (CONFIG_BANNER*, CONFIG_FIBRE_TRANSPORT*) compiles in.
list(APPEND EXTRA_ZEPHYR_MODULES
     ${CMAKE_CURRENT_LIST_DIR}/../../lib)

# Out-of-tree board root (boards/others/fk407m2_zgt6).
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cas_sensor_sim LANGUAGES C CXX)

# --------------------------------------------------------------------------
# Virtual CA engine (plain C, shared with the host golden test in test/).
add_subdirectory(cas)

# --------------------------------------------------------------------------
# Fibre interface codegen: interfaces.hpp / endpoints.cpp /
# static_exports.cpp are generated at build time from
# interface/cas_interface.yaml with the interface generator shipped in the
# Fibre module (single source of truth, same three-template invocation the
# upstream module uses).
set(FIBRE_DIR ${ZEPHYR_FIBRE_MODULE_DIR})
if(NOT FIBRE_DIR)
    # Fallback for setups where the module variable is not exported.
    set(FIBRE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../modules/lib/fibre)
endif()
if(NOT EXISTS ${FIBRE_DIR}/tools/interface_generator.py)
    message(FATAL_ERROR "cas-sim: Fibre module not found at ${FIBRE_DIR} "
                        "(run 'west update')")
endif()
message(STATUS "cas-sim: using Fibre module at ${FIBRE_DIR}")

set(AUTOGEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/autogen)
file(MAKE_DIRECTORY ${AUTOGEN_DIR})

# -W ignore::SyntaxWarning: the upstream generator emits harmless
# SyntaxWarnings for its own regex literals on every invocation.
add_custom_command(
  OUTPUT ${AUTOGEN_DIR}/interfaces.hpp
         ${AUTOGEN_DIR}/endpoints.cpp
         ${AUTOGEN_DIR}/static_exports.cpp
  COMMAND ${Python3_EXECUTABLE} -W ignore::SyntaxWarning
          ${FIBRE_DIR}/tools/interface_generator.py
          --definitions ${CMAKE_CURRENT_SOURCE_DIR}/interface/cas_interface.yaml
          --template ${FIBRE_DIR}/cpp/interfaces_template.j2
          --output ${AUTOGEN_DIR}/interfaces.hpp
  COMMAND ${Python3_EXECUTABLE} -W ignore::SyntaxWarning
          ${FIBRE_DIR}/tools/interface_generator.py
          --definitions ${CMAKE_CURRENT_SOURCE_DIR}/interface/cas_interface.yaml
          --template ${FIBRE_DIR}/cpp/legacy_endpoints_template.j2
          --output ${AUTOGEN_DIR}/endpoints.cpp
  COMMAND ${Python3_EXECUTABLE} -W ignore::SyntaxWarning
          ${FIBRE_DIR}/tools/interface_generator.py
          --definitions ${CMAKE_CURRENT_SOURCE_DIR}/interface/cas_interface.yaml
          --template ${FIBRE_DIR}/cpp/static_exports_template.j2
          --output ${AUTOGEN_DIR}/static_exports.cpp
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/interface/cas_interface.yaml
          ${FIBRE_DIR}/tools/interface_generator.py
          ${FIBRE_DIR}/cpp/interfaces_template.j2
          ${FIBRE_DIR}/cpp/legacy_endpoints_template.j2
          ${FIBRE_DIR}/cpp/static_exports_template.j2
  COMMENT "cas-sim: generating fibre autogen from interface YAML"
  VERBATIM
)

target_sources(app PRIVATE
    src/main.cpp
    src/logo.cpp
    ${AUTOGEN_DIR}/endpoints.cpp
    ${AUTOGEN_DIR}/static_exports.cpp
)

target_include_directories(app PRIVATE src ${AUTOGEN_DIR})

# Mirror the Fibre module's own build settings (see modules/lib/fibre/
# CMakeLists.txt): the Fibre headers expect <stdint.h> to be force-included
# and need RTTI (they use typeid/dynamic_cast but never exceptions).
# CONFIG_CPP_RTTI covers the -frtti part for app sources.
target_compile_options(app PRIVATE -include stdint.h)

# fibre-cpp uses ssize_t (channel_discoverer.cpp) without including
# <sys/types.h>; neither picolibc nor newlib provide it through the headers
# those sources include. zephyr_library_compile_options() passes the module's
# own "-include stdint.h" through a generated options_interface_lib_* helper
# target, so that is where the flag has to be redirected: point it at a
# header that also pulls in <sys/types.h> (src/fibre_libc_fix.h). This
# avoids both modifying the west-managed module checkout and appending a
# second "-include" pair, whose duplicate flag would be dropped by CMake's
# transitive option deduplication.
get_target_property(_fibre_libs ..__modules__lib__fibre LINK_LIBRARIES)
foreach(_fibre_lib ${_fibre_libs})
    if(_fibre_lib MATCHES "^options_interface_lib_")
        get_target_property(_fibre_opts ${_fibre_lib} INTERFACE_COMPILE_OPTIONS)
        list(FIND _fibre_opts "stdint.h" _fibre_stdint_idx)
        if(NOT _fibre_stdint_idx EQUAL -1)
            list(REMOVE_AT _fibre_opts ${_fibre_stdint_idx})
            list(INSERT _fibre_opts ${_fibre_stdint_idx}
                 "${CMAKE_CURRENT_LIST_DIR}/src/fibre_libc_fix.h")
            set_target_properties(${_fibre_lib}
                PROPERTIES INTERFACE_COMPILE_OPTIONS "${_fibre_opts}")
        endif()
    endif()
endforeach()
unset(_fibre_libs)
unset(_fibre_lib)
unset(_fibre_opts)
unset(_fibre_stdint_idx)

# Add APP_VERSION_* / PLATFORM_VERSION_* compile definitions once the app
# target exists, and generate the version description headers that firmware
# code includes ("app_version.h" / "platform_version.h").
git_version_finalize()
platform_version_finalize()
version_header_generate(${CMAKE_CURRENT_BINARY_DIR}/generated)
