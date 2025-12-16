# File: config/boards/shields/nice_view/CMakeLists.txt
# Build configuration for nice_view with temperature

# Thêm custom peripheral_status với temperature
target_sources(app PRIVATE widgets/peripheral_status.c)
