# File: zmk-config/app/nice_view_custom_widgets/CMakeLists.txt

# Khai báo đây là một thư viện Zephyr/ZMK độc lập (Out-of-Tree Module)
zephyr_library()

# Lệnh GHI ĐÈ file peripheral_status.c gốc

# Target name: zmk_nice_view_widgets là tên Target (thư viện CMake) của module Nice!View Widgets gốc trong ZMK
target_sources_override(zmk_nice_view_widgets
    PRIVATE 
        # Đường dẫn file GỐC cần thay thế (Relate to ZMK source root)
        widgets/peripheral_status.c
    WITH 
        # Tên file custom bạn đang cung cấp (phải nằm trong thư mục này)
        peripheral_status.c
)

# Thêm các file mã nguồn C mới (ví dụ: temperature.c)
# Bạn nên sử dụng zephyr_library_sources() để thêm các tệp mới vào thư viện này
# zephyr_library_sources(
    # temperature.c
    #Nếu bạn có thêm các file C khác cho widget custom, thêm vào đây
# )

# Nếu bạn có thêm các thư mục include, bạn nên thêm bằng lệnh này
# target_include_directories(zmk_nice_view_widgets PRIVATE include)