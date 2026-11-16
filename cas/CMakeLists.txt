# CASensorSim virtual CA engine -- platform-independent core library.
#
# include/ holds the public headers (cas_core.h, cas_fault.h,
# cas_preset.h); src/ the implementation. In plain add_subdirectory()
# context the sources are compiled into the app image directly
# (zephyr_library() is Zephyr-module-only and warns in app mode), which
# keeps the library structure without the module ceremony. The core is
# plain C by design: the host golden test (../test) compiles the same
# sources with a Makefile, no Zephyr needed.

target_sources(app PRIVATE
    src/cas_core.c
    src/cas_fault.c
)
target_include_directories(app PRIVATE include)
