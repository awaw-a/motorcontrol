add_library(__memory_StaticArena INTERFACE)

target_include_directories(__memory_StaticArena INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(memory::StaticArena ALIAS __memory_StaticArena)
