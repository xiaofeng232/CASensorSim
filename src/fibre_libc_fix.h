/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Replacement for the "-include stdint.h" that the Fibre module applies to
 * its own sources (see modules/lib/fibre/CMakeLists.txt). Shared with
 * tests/fibre-test (same file).
 *
 * fibre-cpp uses POSIX types (ssize_t in channel_discoverer.cpp) without
 * including <sys/types.h>; neither picolibc nor newlib provide it through
 * the headers those sources include. This header force-includes both, so a
 * single "-include" flag covers the module's original stdint.h requirement
 * and the missing POSIX types at once.
 *
 * The indirection exists because a second "-include <file>" pair appended to
 * the module target gets broken up by CMake's compile option
 * deduplication, which would leave the module's own stdint.h argument bare
 * (parsed as an input file).
 */

#include <stdint.h>
#include <sys/types.h>
