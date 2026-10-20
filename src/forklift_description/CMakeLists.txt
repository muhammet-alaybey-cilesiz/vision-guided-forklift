cmake_minimum_required(VERSION 3.8)
project(forklift_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Yalnızca veri paketi: tanım, parametreler, launch ve RViz yapılandırması
install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# Gövde çıkartmaları (uyarı şeridi, yan etiket): build sırasında
# forklift_params.yaml → appearance.decals'tan üretilir, kaynak ağaca yazılmaz
# (DECISIONS D-54). URDF package://forklift_description/meshes/decals/*.obj ile
# kullanır; gz bunu model://forklift_description olarak çözer (package.xml export).
set(DECALS_DIR ${CMAKE_CURRENT_BINARY_DIR}/meshes/decals)
add_custom_command(
  OUTPUT ${DECALS_DIR}/rear_hazard.obj ${DECALS_DIR}/side_label.obj
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_decals.py
    --params ${CMAKE_CURRENT_SOURCE_DIR}/config/forklift_params.yaml
    --out ${DECALS_DIR}
  DEPENDS scripts/generate_decals.py config/forklift_params.yaml
  COMMENT "Gövde çıkartmaları üretiliyor"
  VERBATIM
)
add_custom_target(decals ALL DEPENDS ${DECALS_DIR}/rear_hazard.obj ${DECALS_DIR}/side_label.obj)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/meshes DESTINATION share/${PROJECT_NAME})

ament_package()
