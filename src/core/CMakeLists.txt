set (core_source_files clip.cpp clip_player.cpp floor_extractor.cpp frame_observer.cpp global_options.cpp icp_framed_tracker.cpp kinect_interface.cpp producer_consumer_queue.cpp selection_cube.cpp template.cpp tracker.cpp tracker.cpp update_signal.cpp utils.cpp)

add_library (core ${core_source_files})

target_link_libraries (core ${PCL_LIBRARIES} ${Boost_LIBRARIES})
