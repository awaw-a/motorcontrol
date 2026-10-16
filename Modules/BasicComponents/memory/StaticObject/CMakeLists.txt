add_library(__memory_StaticObject INTERFACE)

target_include_directories(__memory_StaticObject INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(memory::StaticObject ALIAS __memory_StaticObject)
