add_library(__protocol_UartRxSync INTERFACE)

target_include_directories(__protocol_UartRxSync INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(__protocol_UartRxSync INTERFACE memory::DMABuffer)
target_link_libraries(__protocol_UartRxSync INTERFACE services::Watchdog)
target_link_libraries(__protocol_UartRxSync INTERFACE stm32cubemx)

# alias for external use
add_library(protocol::UartRxSync ALIAS __protocol_UartRxSync)
