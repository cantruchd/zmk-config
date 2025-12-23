# File: config/CMakeLists.txt (root của config folder)

# Override nice_view peripheral_status widget
if(CONFIG_ZMK_DISPLAY)
    if(CONFIG_SHIELD_NICE_VIEW OR CONFIG_SHIELD_NICE_VIEW_ADAPTER)
        # Get absolute path
        set(CUSTOM_WIDGET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/boards/shields/nice_view/widgets)
        
        if(EXISTS ${CUSTOM_WIDGET_DIR}/peripheral_status.c)
            message(STATUS "Using custom peripheral_status.c from config")
            target_sources(app PRIVATE ${CUSTOM_WIDGET_DIR}/peripheral_status.c)
            target_include_directories(app PRIVATE ${CUSTOM_WIDGET_DIR})
        endif()
    endif()
endif()
