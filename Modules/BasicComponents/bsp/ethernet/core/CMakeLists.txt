add_library(__EthernetPHY_Core INTERFACE)

target_include_directories(__EthernetPHY_Core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(EthernetPHY::Core ALIAS __EthernetPHY_Core)
