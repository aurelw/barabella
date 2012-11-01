set (gui_source_files view3d.cpp)

include_directories (../core/)

add_library (gui ${gui_source_files})

target_link_libraries (gui core)
