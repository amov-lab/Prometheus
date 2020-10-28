set(MODEL_NAME vrc_heightmap_1)
set (files 
  model.config 
  model.sdf
)

add_subdirectory(materials)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/gazebo_models/environments/${MODEL_NAME}/)
