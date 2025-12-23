# File: CMakeLists.txt (root của zmk-config, không phải config/)

# Add config directory to build
if(CONFIG_SHIELD_NICE_VIEW OR CONFIG_SHIELD_NICE_VIEW_ADAPTER)
    message(STATUS "=== Loading custom nice_view widgets ===")
    
    set(CUSTOM_WIDGET_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config/boards/shields/nice_view/widgets/peripheral_status.c)
    
    if(EXISTS ${CUSTOM_WIDGET_PATH})
        message(STATUS "=== FOUND: ${CUSTOM_WIDGET_PATH} ===")
        target_sources(app PRIVATE ${CUSTOM_WIDGET_PATH})
        target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/config/boards/shields/nice_view/widgets)
    else()
        message(WARNING "=== NOT FOUND: ${CUSTOM_WIDGET_PATH} ===")
    endif()
endif()
