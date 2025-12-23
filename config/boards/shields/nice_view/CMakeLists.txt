# File: config/boards/shields/nice_view/CMakeLists.txt

# Kiểm tra nếu file này đang được build cho shield nice_view
#if (CONFIG_SHIELD_NICE_VIEW)
    # Thêm đường dẫn include để trình biên dịch ưu tiên tìm file header local của bạn
#    zephyr_include_directories(widgets)

    # Chỉ định rõ ràng file nguồn của bạn
    # Zephyr sẽ tự động xử lý việc trùng lặp nếu được cấu hình đúng trong module
#    zephyr_library_sources(widgets/peripheral_status.c)
#endif()
