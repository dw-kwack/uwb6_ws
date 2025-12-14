add_library(sensors_lib SHARED
  laser/laser.cpp
  laser/beam_model.cpp
  laser/likelihood_field_model.cpp
  laser/likelihood_field_model_prob.cpp
  ########## 2025.08.02 by Noh begin
  laser/tag_anchor_model.cpp
  ########## end
)
# map_update_cspace
target_link_libraries(sensors_lib pf_lib map_lib)

########## 2025.08.02 by Noh begin
# Add target dependencies
ament_target_dependencies(sensors_lib
  rclcpp
  geometry_msgs
)
########## end

install(TARGETS
  sensors_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
