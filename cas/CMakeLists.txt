# CASensorSim virtual CA engine -- platform-independent core library.
#
# include/ holds the public headers (cas_core.h, cas_preset.h); src/ the
# implementation. zephyr_library() keeps the library inside the Zephyr
# build (same flags/includes as the rest of the image). The core is
# plain C by design: the host golden test (../test) compiles the same
# sources with a Makefile, no Zephyr needed.

zephyr_library()
zephyr_library_sources(
    src/cas_core.c
)
zephyr_include_directories(include)
