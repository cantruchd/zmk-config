# CMakeLists.txt
# Build configuration for battery_monitor shield
#
# Location: config/boards/shields/battery_monitor/CMakeLists.txt
#
# This file tells the build system to compile battery_monitor.c

# Add custom C source file to build
target_sources(app PRIVATE battery_monitor.c)