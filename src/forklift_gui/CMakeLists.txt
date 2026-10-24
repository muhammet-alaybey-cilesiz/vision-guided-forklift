cmake_minimum_required(VERSION 3.16)
project(forklift_gui LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(forklift_interfaces REQUIRED)

# Sistem Qt 6.4 (apt; DECISIONS D-22). AUTOMOC ve GNUInstallDirs ayarlanir.
find_package(Qt6 6.4 REQUIRED COMPONENTS
  Core Gui Qml Quick QuickControls2 Multimedia Concurrent)
qt_standard_project_setup()

# QML modullerinin derleme ciktisi (qmldir, qmltypes) tek kokte;
# qmlcachegen ve qmllint bu yolu ice aktarma yolu olarak kullanir.
set(QT_QML_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/qml)

# ------------------------------------------------------------------ core --
# Yalniz Qt'ye bagimli; hicbir ROS basligi icermez (ROS include yolu yok).
set(CORE_SOURCES
  include/forklift_gui/core/SteadyClock.hpp
  include/forklift_gui/core/Types.hpp
  include/forklift_gui/core/GuiConfig.hpp
  src/core/GuiConfig.cpp
  include/forklift_gui/core/ports/IRobotCommandPort.hpp
  include/forklift_gui/core/ports/IRobotStatePort.hpp
  include/forklift_gui/core/ports/ICameraPort.hpp
  include/forklift_gui/core/ports/ILogPort.hpp
  include/forklift_gui/core/ports/IAutonomyPort.hpp
  include/forklift_gui/core/TeleopController.hpp
  src/core/TeleopController.cpp
  include/forklift_gui/core/ForkController.hpp
  src/core/ForkController.cpp
  include/forklift_gui/core/SafetyController.hpp
  src/core/SafetyController.cpp
  include/forklift_gui/core/TelemetryModel.hpp
  src/core/TelemetryModel.cpp
  include/forklift_gui/core/TopicHealthModel.hpp
  src/core/TopicHealthModel.cpp
  include/forklift_gui/core/LogModel.hpp
  src/core/LogModel.cpp
  include/forklift_gui/core/CameraStream.hpp
  src/core/CameraStream.cpp
  include/forklift_gui/core/DatasetRecorder.hpp
  src/core/DatasetRecorder.cpp
  include/forklift_gui/core/AutonomyController.hpp
  src/core/AutonomyController.cpp
  include/forklift_gui/core/Backend.hpp
  src/core/Backend.cpp
)

qt_add_library(forklift_gui_core STATIC)
qt_add_qml_module(forklift_gui_core
  URI Forklift.Core
  VERSION 1.0
  RESOURCE_PREFIX /qt/qml
  SOURCES ${CORE_SOURCES}
)
target_include_directories(forklift_gui_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Qt 6.4 qmltyperegistrar basliklari yalniz dosya adiyla dahil eder
# (#include <Backend.hpp>); uretilen kayit dosyasi icin.
target_include_directories(forklift_gui_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/forklift_gui/core)
target_link_libraries(forklift_gui_core PUBLIC
  Qt6::Core Qt6::Gui Qt6::Qml Qt6::Multimedia Qt6::Concurrent)

# ------------------------------------------------------------------ mock --
add_library(forklift_gui_mock STATIC
  include/forklift_gui/mock/MockRobotAdapter.hpp
  src/mock/MockRobotAdapter.cpp
)
target_link_libraries(forklift_gui_mock PUBLIC forklift_gui_core)

# ------------------------------------------------------------------- ros --
add_library(forklift_gui_ros STATIC
  include/forklift_gui/ros/RosParamLoader.hpp
  src/ros/RosParamLoader.cpp
  include/forklift_gui/ros/ImageConverter.hpp
  src/ros/ImageConverter.cpp
  include/forklift_gui/ros/RosBridge.hpp
  src/ros/RosBridge.cpp
)
target_link_libraries(forklift_gui_ros PUBLIC
  forklift_gui_core
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${rosgraph_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${vision_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${forklift_interfaces_TARGETS})

# ----------------------------------------------------------- uygulama --
set(QML_FILES
  qml/Main.qml
  qml/UiState.qml
  qml/theme/Theme.qml
  qml/components/PanelFrame.qml
  qml/components/StatusLed.qml
  qml/components/ValueTile.qml
  qml/components/VirtualJoystick.qml
  qml/components/CameraView.qml
  qml/components/DetectionOverlay.qml
  qml/components/EStopButton.qml
  qml/components/ForkGauge.qml
  qml/components/OdomTrail.qml
  qml/components/HelpOverlay.qml
  qml/components/ThemedTabButton.qml
  qml/panels/TopBar.qml
  qml/panels/CameraPanel.qml
  qml/panels/DrivePanel.qml
  qml/panels/ForkPanel.qml
  qml/panels/TelemetryPanel.qml
  qml/panels/HealthPanel.qml
  qml/panels/LogPanel.qml
  qml/panels/DatasetPanel.qml
  qml/panels/BottomTabs.qml
  qml/panels/AutonomyPanel.qml
)
set_source_files_properties(qml/theme/Theme.qml qml/UiState.qml
  PROPERTIES QT_QML_SINGLETON_TYPE TRUE)

qt_add_executable(forklift_gui
  src/main.cpp
  src/app/KeyRouter.hpp
  src/app/KeyRouter.cpp
)
# QML dosyalari qrc'ye gomulur; calisma zamaninda dosya yolu aranmaz.
qt_add_qml_module(forklift_gui
  URI Forklift.Gui
  VERSION 1.0
  RESOURCE_PREFIX /qt/qml
  QML_FILES ${QML_FILES}
)
target_include_directories(forklift_gui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(forklift_gui PRIVATE
  forklift_gui_core
  forklift_gui_coreplugin
  forklift_gui_mock
  forklift_gui_ros
  ament_index_cpp::ament_index_cpp
  Qt6::Quick
  Qt6::QuickControls2)

install(TARGETS forklift_gui RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# Uctan uca testler: ros2 run forklift_gui integration_test.py (sim + GUI)
install(PROGRAMS test/integration/integration_test.py
  DESTINATION lib/${PROJECT_NAME})

# ----------------------------------------------------------------- test --
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_test REQUIRED)
  find_package(Qt6 REQUIRED COMPONENTS Test)

  # Core birim testleri (QtTest, mock adaptor, offscreen); U1-U8 ve
  # ek testler (telemetri kisma/RTF, log modeli, kamera akisi).
  set(CORE_TESTS teleop safety fork health dataset telemetry log camera autonomy)
  foreach(name IN LISTS CORE_TESTS)
    qt_add_executable(test_${name}
      test/core/test_${name}.cpp
      test/core/TestSupport.hpp)
    target_include_directories(test_${name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/test/core)
    target_compile_definitions(test_${name} PRIVATE
      FORKLIFT_GUI_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/config/gui_params.yaml")
    target_link_libraries(test_${name} PRIVATE
      forklift_gui_mock forklift_gui_ros Qt6::Test)
    set(result_file
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_${name}.xml")
    ament_add_test(test_${name}
      COMMAND $<TARGET_FILE:test_${name}>
        -o ${result_file},junitxml -o -,txt
      RESULT_FILE ${result_file}
      ENV QT_QPA_PLATFORM=offscreen
      TIMEOUT 120)
  endforeach()

  # qmllint 6.4.2: Qt'nin urettigi hedef dogru ice aktarma yollarini kurar.
  ament_add_test(qmllint
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}
      --target forklift_gui_qmllint
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300)
endif()

ament_package()
