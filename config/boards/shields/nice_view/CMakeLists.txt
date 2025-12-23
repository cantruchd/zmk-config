# File: config/boards/shields/nice_view/CMakeLists.txt
# Override nice_view peripheral_status widget

# Thêm custom peripheral_status.c (sẽ override file gốc)
target_sources(app PRIVATE widgets/peripheral_status.c)
