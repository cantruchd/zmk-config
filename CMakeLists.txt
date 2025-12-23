# File: CMakeLists.txt (root của zmk-config)

if(CONFIG_SHIELD_NICE_VIEW OR CONFIG_SHIELD_NICE_VIEW_ADAPTER)
    message(STATUS "=== Loading custom nice_view widgets ===")
    
    set(CUSTOM_WIDGET_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config/boards/shields/nice_view/widgets/peripheral_status.c)
    
    if(EXISTS ${CUSTOM_WIDGET_PATH})
        message(STATUS "=== FOUND: ${CUSTOM_WIDGET_PATH} ===")
        
        # Add custom widget source
        target_sources(app PRIVATE ${CUSTOM_WIDGET_PATH})
        
        # Find ZMK nice_view widgets path
        # GitHub Actions: /tmp/zmk-config/zmk/app/boards/shields/nice_view/widgets
        # The app target is defined in zmk/app, so we can use its source dir
        get_target_property(APP_SOURCE_DIR app SOURCE_DIR)
        set(ZMK_NICE_VIEW_WIDGETS ${APP_SOURCE_DIR}/boards/shields/nice_view/widgets)
        
        if(EXISTS ${ZMK_NICE_VIEW_WIDGETS}/util.h)
            message(STATUS "=== FOUND util.h at: ${ZMK_NICE_VIEW_WIDGETS} ===")
            target_include_directories(app PRIVATE ${ZMK_NICE_VIEW_WIDGETS})
        else()
            message(WARNING "=== util.h NOT FOUND at: ${ZMK_NICE_VIEW_WIDGETS} ===")
            message(STATUS "=== APP_SOURCE_DIR: ${APP_SOURCE_DIR} ===")
        endif()
    else()
        message(WARNING "=== NOT FOUND: ${CUSTOM_WIDGET_PATH} ===")
    endif()
endif()
