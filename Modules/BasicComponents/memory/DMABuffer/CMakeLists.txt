add_library(__memory_DMABuffer INTERFACE)

target_include_directories(__memory_DMABuffer INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(memory::DMABuffer ALIAS __memory_DMABuffer)
