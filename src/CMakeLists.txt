add_executable(weight_tracker weighttracker.cpp kinect_interface.cpp view3d.cpp floor_extractor.cpp utils.cpp clip.cpp global_options.cpp selection_cube.cpp update_signal.cpp)

target_link_libraries(weight_tracker ${PCL_LIBRARIES} ${Boost_LIBRARIES})

