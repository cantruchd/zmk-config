# boards/shields/nice_view_custom/CMakeLists.txt

if (CONFIG_SHIELD_NICE_VIEW_CUSTOM)
	zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)

    # 2. Chỉ định thư mục chứa các file .h (header) của bạn
    zephyr_library_include_directories(widgets)
    
    # 3. Thêm các file nguồn .c vào quá trình biên dịch
    zephyr_library_sources(custom_status_screen.c)
    zephyr_library_sources(widgets/status.c)
	zephyr_library_sources(widgets/util.c)
	zephyr_library_sources(widgets/art.c)
	zephyr_library_sources(widgets/bolt.c)
	zephyr_library_sources(widgets/peripheral_status.c)

    # 4. QUAN TRỌNG: Kết nối với LVGL để file .c đọc được #include <lvgl.h>
    zephyr_library_link_libraries(LVGL)
endif()