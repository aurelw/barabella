add_subdirectory (core)
add_subdirectory (gui)

include_directories (./core/)
include_directories (./gui/)

add_executable (weight_tracker weighttracker.cpp barabella_app.cpp)

target_link_libraries (weight_tracker gui)

