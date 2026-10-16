add_library(EthernetUdpSocket STATIC
    "./udp_socket.cpp"
)

target_include_directories(EthernetUdpSocket
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(EthernetUdpSocket PUBLIC EthernetPHY::Core)
target_link_libraries(EthernetUdpSocket PUBLIC FreeRTOS)
target_link_libraries(EthernetUdpSocket PUBLIC LwIP)
target_link_libraries(EthernetUdpSocket PUBLIC stm32cubemx)

# alias for external use
add_library(Ethernet::UdpSocket ALIAS EthernetUdpSocket)
