cmake_minimum_required(VERSION 3.8)
project(forklift_gazebo)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Dünya, modeller, köprü yapılandırması, launch dosyaları ve SDF xacro'ları
install(
  DIRECTORY config launch models worlds xacro
  DESTINATION share/${PROJECT_NAME}
)

# ArUco dokuları ve markers.yaml: build sırasında üretilir, kaynak ağaca
# yazılmaz (DECISIONS D-47). Dokular model://aruco_markers/materials/textures
# altına kurulur (models/ dizini GZ_SIM_RESOURCE_PATH'te, package.xml export).
set(ARUCO_MODEL_DIR ${CMAKE_CURRENT_BINARY_DIR}/models/aruco_markers)
set(MARKERS_YAML ${CMAKE_CURRENT_BINARY_DIR}/config/markers.yaml)
add_custom_command(
  OUTPUT ${MARKERS_YAML}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_aruco_textures.py
    --params ${CMAKE_CURRENT_SOURCE_DIR}/config/station_params.yaml
    --stations ${CMAKE_CURRENT_SOURCE_DIR}/config/stations.yaml
    --textures ${ARUCO_MODEL_DIR}/materials/textures
    --markers ${MARKERS_YAML}
  DEPENDS
    scripts/generate_aruco_textures.py
    scripts/station_layout.py
    config/station_params.yaml
    config/stations.yaml
  COMMENT "ArUco dokuları ve markers.yaml üretiliyor"
  VERBATIM
)
add_custom_target(aruco_textures ALL DEPENDS ${MARKERS_YAML})
install(DIRECTORY ${ARUCO_MODEL_DIR} DESTINATION share/${PROJECT_NAME}/models)
install(FILES ${MARKERS_YAML} DESTINATION share/${PROJECT_NAME}/config)

# Kabul testleri: ros2 run forklift_gazebo acceptance_test.py | station_acceptance.py
install(
  PROGRAMS
    scripts/acceptance_test.py
    scripts/station_acceptance.py
    scripts/generate_aruco_textures.py
  DESTINATION lib/${PROJECT_NAME}
)
# Ortak yerleşim modülü (launch ve testler içe aktarır)
install(
  FILES scripts/station_layout.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
