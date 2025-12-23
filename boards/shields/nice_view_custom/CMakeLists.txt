# boards/shields/nice_view_custom/CMakeLists.txt

if (CONFIG_SHIELD_NICE_VIEW_CUSTOM)
    zephyr_include_directories(widgets)
    target_sources(app PRIVATE 
        custom_status_screen.c 
        widgets/peripheral_status.c
    )
endif()