add_library(EthernetPHYLwipPlatform STATIC
    "./LwipPlatform.cpp"
)

target_include_directories(EthernetPHYLwipPlatform
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(EthernetPHYLwipPlatform PUBLIC EthernetPHY::Core)
target_link_libraries(EthernetPHYLwipPlatform PUBLIC FreeRTOS)
target_link_libraries(EthernetPHYLwipPlatform PUBLIC LwIP)
target_link_libraries(EthernetPHYLwipPlatform PUBLIC stm32cubemx)

# alias for external use
add_library(EthernetPHY::LwipPlatform ALIAS EthernetPHYLwipPlatform)
