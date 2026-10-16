add_library(EthernetPHYDP83822 STATIC
    "./DP83822Phy.cpp"
)

target_include_directories(EthernetPHYDP83822
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(EthernetPHYDP83822 PUBLIC EthernetPHY::Core)
target_link_libraries(EthernetPHYDP83822 PUBLIC bsp::GPIO_Driver)
target_link_libraries(EthernetPHYDP83822 PUBLIC stm32cubemx)

# alias for external use
add_library(EthernetPHY::DP83822 ALIAS EthernetPHYDP83822)
