# boards/shields/nice_view_custom/CMakeLists.txt

if (CONFIG_SHIELD_NICE_VIEW_CUSTOM)
    zephyr_include_directories(widgets)
    target_sources(app PRIVATE 
        custom_status_screen.c 
        widgets/peripheral_status.c
    )
endif()

if(CONFIG_ZMK_DISPLAY AND CONFIG_NICE_VIEW_WIDGET_STATUS)
  zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
endif()