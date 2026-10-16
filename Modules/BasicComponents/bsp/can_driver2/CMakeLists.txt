add_library(BspCANDriver2 STATIC
    "./can_driver.cpp"
    "./can_driver_bxcan.cpp"
    "./can_driver_fdcan.cpp"
)

target_include_directories(BspCANDriver2
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(BspCANDriver2 PUBLIC libs::RingBuffer)
target_link_libraries(BspCANDriver2 PUBLIC stm32cubemx)
target_link_libraries(BspCANDriver2 PUBLIC utils)

# alias for external use
add_library(bsp::CANDriver2 ALIAS BspCANDriver2)
