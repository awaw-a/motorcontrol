add_library(VelocityProfileSCurve STATIC
    "./src/s_curve.cpp"
)

target_include_directories(VelocityProfileSCurve
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# link dependencies if any
target_link_libraries(VelocityProfileSCurve PUBLIC VelocityProfile::Core)

# alias for external use
add_library(VelocityProfile::SCurve ALIAS VelocityProfileSCurve)
