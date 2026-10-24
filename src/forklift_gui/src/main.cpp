// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

// forklift_gui composition root: argümanlar, yapılandırma, adaptör seçimi
// (ROS ya da --mock), Backend kurulumu, QML yükleme, sinyal işleme ve
// düzenli kapanış.

#include <QtQml/qqmlextensionplugin.h>
#include <sys/socket.h>
#include <unistd.h>

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QSocketNotifier>
#include <QTimer>
#include <algorithm>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "app/KeyRouter.hpp"
#include "forklift_gui/core/Backend.hpp"
#include "forklift_gui/mock/MockRobotAdapter.hpp"
#include "forklift_gui/ros/RosBridge.hpp"
#include "forklift_gui/ros/RosParamLoader.hpp"
#include "rclcpp/rclcpp.hpp"

Q_IMPORT_QML_PLUGIN(Forklift_CorePlugin)

namespace
{

constexpr const char * kNodeName = "forklift_gui";
constexpr const char * kMainQml = "qrc:/qt/qml/Forklift/Gui/qml/Main.qml";

int g_signalFd[2] = {-1, -1};

// Sinyal işleyicide yalnız async-signal-safe write(); asıl iş Qt döngüsünde.
void onSignal(int signalNumber)
{
  (void)signalNumber;
  const char c = 1;
  const ssize_t n = ::write(g_signalFd[1], &c, 1);
  (void)n;
}

struct AppArgs
{
  bool mock{false};
  bool help{false};
  std::string ownParamsFile;  // ROS dışı --params-file
  std::string anyParamsFile;  // son --params-file (--ros-args içi dahil)
  bool rosParamsFile{false};  // --ros-args içinde --params-file var mı
  QString screenshot;
  double screenshotDelay{3.0};
  QString error;
};

void printUsage()
{
  std::fputs(
    "Kullanım: forklift_gui [--mock] [--params-file <yaml>] [--screenshot <png>]\n"
    "                       [--screenshot-delay <s>] [--ros-args ...]\n"
    "  --mock              ROS olmadan sahte verilerle aç\n"
    "  --params-file       gui_params.yaml (varsayılan: paketin config/ dosyası)\n"
    "  --screenshot        gecikmeden sonra pencere görüntüsünü PNG kaydet ve çık\n"
    "  --screenshot-delay  ekran görüntüsü gecikmesi [s] (varsayılan 3)\n",
    stdout);
}

AppArgs parseArgs(const std::vector<std::string> & nonRos, const std::vector<std::string> & all)
{
  AppArgs args;
  for (size_t i = 1; i < nonRos.size(); ++i) {
    const std::string & a = nonRos[i];
    auto value = [&](std::string & out) {
      if (i + 1 >= nonRos.size()) {
        args.error = QStringLiteral("%1 bir değer bekliyor").arg(QString::fromStdString(a));
        return false;
      }
      out = nonRos[++i];
      return true;
    };
    std::string v;
    if (a == "--mock") {
      args.mock = true;
    } else if (a == "--help" || a == "-h") {
      args.help = true;
    } else if (a == "--screenshot") {
      if (value(v)) {
        args.screenshot = QString::fromStdString(v);
      }
    } else if (a == "--screenshot-delay") {
      if (value(v)) {
        args.screenshotDelay = std::stod(v);
      }
    } else if (a == "--params-file") {
      if (value(v)) {
        args.ownParamsFile = v;
      }
    }
  }
  int total = 0;
  for (size_t i = 1; i + 1 < all.size(); ++i) {
    if (all[i] == "--params-file") {
      args.anyParamsFile = all[i + 1];
      ++total;
    }
  }
  const int own = static_cast<int>(std::count(nonRos.begin(), nonRos.end(), "--params-file"));
  args.rosParamsFile = total > own;
  return args;
}

std::string defaultParamsFile()
{
  return ament_index_cpp::get_package_share_directory(kNodeName) + "/config/gui_params.yaml";
}

bool installSignalHandlers(QCoreApplication & app)
{
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, g_signalFd) != 0) {
    return false;
  }
  auto * notifier = new QSocketNotifier(g_signalFd[0], QSocketNotifier::Read, &app);
  QObject::connect(notifier, &QSocketNotifier::activated, &app, [&app]() {
    char c = 0;
    const ssize_t n = ::read(g_signalFd[0], &c, 1);
    (void)n;
    app.quit();  // ROS ve Qt aynı düzenli kapanış yolunu izler
  });
  struct sigaction action
  {
  };
  action.sa_handler = onSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESTART;
  return ::sigaction(SIGINT, &action, nullptr) == 0 && ::sigaction(SIGTERM, &action, nullptr) == 0;
}

// Moda göre veri kaynakları; Backend'den ÖNCE kurulur, SONRA yıkılır.
struct Runtime
{
  std::unique_ptr<forklift_gui::mock::MockRobotAdapter> mock;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<forklift_gui::ros_adapter::RosBridge> bridge;
  forklift_gui::Backend::Ports ports;
  bool rosInitialized{false};

  void start()
  {
    if (mock) {
      mock->start();
    }
    if (bridge) {
      bridge->start();
    }
  }

  void stop()
  {
    if (mock) {
      mock->stop();
    }
    if (bridge) {
      bridge->stop();  // executor iptal + thread join
    }
  }

  ~Runtime()
  {
    stop();
    bridge.reset();
    node.reset();
    mock.reset();
    if (rosInitialized) {
      rclcpp::shutdown();
    }
  }
};

// ROS modu: rclcpp başlatılır, parametreler düğümden okunur (D-29).
void setupRos(
  Runtime & rt, const AppArgs & args, const std::vector<std::string> & all,
  forklift_gui::GuiConfig & config)
{
  std::vector<std::string> rosArgs = all;
  // --params-file yalnız ROS dışında verildiyse ya da hiç verilmediyse en
  // başa eklenir; sonraki -p / --params-file onu ezer (rcl: son değer kazanır).
  std::string params = args.ownParamsFile;
  if (params.empty() && !args.rosParamsFile) {
    params = defaultParamsFile();
  }
  if (!params.empty()) {
    rosArgs.insert(rosArgs.begin() + 1, {"--ros-args", "--params-file", params, "--"});
  }
  std::vector<char *> rosArgv;
  for (std::string & s : rosArgs) {
    rosArgv.push_back(s.data());
  }
  rclcpp::init(
    static_cast<int>(rosArgv.size()), rosArgv.data(), rclcpp::InitOptions(),
    rclcpp::SignalHandlerOptions::None);
  rt.rosInitialized = true;
  rt.node = std::make_shared<rclcpp::Node>(kNodeName);
  config = forklift_gui::ros_adapter::loadConfig(*rt.node);
  rt.bridge = std::make_unique<forklift_gui::ros_adapter::RosBridge>(rt.node, config);
  rt.ports.command = rt.bridge->commandPort();
  rt.ports.state = rt.bridge->statePort();
  rt.ports.camera = rt.bridge->cameraPort();
  rt.ports.log = rt.bridge->logPort();
  rt.ports.autonomy = rt.bridge->autonomyPort();
}

// --mock: ROS grafiği yok; aynı YAML rclcpp ayrıştırıcısıyla okunur.
void setupMock(Runtime & rt, const AppArgs & args, forklift_gui::GuiConfig & config)
{
  std::string params = args.ownParamsFile;
  if (params.empty()) {
    params = args.anyParamsFile.empty() ? defaultParamsFile() : args.anyParamsFile;
  }
  config = forklift_gui::ros_adapter::loadConfigFromYaml(params, std::string("/") + kNodeName);
  rt.mock =
    std::make_unique<forklift_gui::mock::MockRobotAdapter>(config, forklift_gui::steadyNowNs);
  rt.ports.command = rt.mock->command();
  rt.ports.state = rt.mock->state();
  rt.ports.camera = rt.mock->camera();
  rt.ports.log = rt.mock->log();
  rt.ports.autonomy = rt.mock->autonomy();
}

}  // namespace

int main(int argc, char ** argv)
{
  using forklift_gui::Backend;
  using forklift_gui::GuiConfig;

  const std::vector<std::string> all(argv, argv + argc);
  const std::vector<std::string> nonRos = rclcpp::remove_ros_arguments(argc, argv);
  const AppArgs args = parseArgs(nonRos, all);
  if (args.help) {
    printUsage();
    return 0;
  }
  if (!args.error.isEmpty()) {
    std::fprintf(stderr, "forklift_gui: %s\n", qPrintable(args.error));
    printUsage();
    return 2;
  }

  // Qt yalnız ROS dışı argümanları görür.
  std::vector<std::string> qtArgStore = nonRos;
  std::vector<char *> qtArgv;
  for (std::string & s : qtArgStore) {
    qtArgv.push_back(s.data());
  }
  int qtArgc = static_cast<int>(qtArgv.size());
  QGuiApplication app(qtArgc, qtArgv.data());
  QGuiApplication::setApplicationName(QString::fromLatin1(kNodeName));

  Runtime rt;
  GuiConfig config;
  try {
    if (args.mock) {
      setupMock(rt, args, config);
    } else {
      setupRos(rt, args, all, config);
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "forklift_gui: %s\n", e.what());
    return 1;
  }

  Backend backend(config, rt.ports, args.mock, forklift_gui::steadyNowNs);
  Backend::setInstance(&backend);

  QQmlApplicationEngine engine;
  // Qt 6.4'te qrc:/qt/qml varsayılan içe aktarma yolu değil (D-28).
  engine.addImportPath(QStringLiteral("qrc:/qt/qml"));
  engine.load(QUrl(QString::fromLatin1(kMainQml)));
  if (engine.rootObjects().isEmpty()) {
    std::fprintf(stderr, "forklift_gui: QML yüklenemedi\n");
    return 1;
  }
  auto * window = qobject_cast<QQuickWindow *>(engine.rootObjects().constFirst());
  if (!window) {
    std::fprintf(stderr, "forklift_gui: kök nesne bir pencere değil\n");
    return 1;
  }

  forklift_gui::KeyRouter router(window, &backend);
  QObject::connect(window, &QWindow::activeChanged, &backend, [&backend, window]() {
    backend.setWindowActive(window->isActive());
  });
  backend.setWindowActive(window->isActive());

  if (!installSignalHandlers(app)) {
    std::fprintf(stderr, "forklift_gui: sinyal işleyicileri kurulamadı\n");
  }

  if (!args.screenshot.isEmpty()) {
    const QString path = args.screenshot;
    QTimer::singleShot(static_cast<int>(args.screenshotDelay * 1000.0), window, [window, path]() {
      const bool ok = window->grabWindow().save(path);
      std::fprintf(
        ok ? stdout : stderr, "forklift_gui: ekran görüntüsü %s: %s\n",
        ok ? "kaydedildi" : "KAYDEDİLEMEDİ", qPrintable(path));
      QCoreApplication::exit(ok ? 0 : 3);
    });
  }

  backend.start();
  rt.start();
  const int rc = app.exec();

  // Düzenli kapanış: önce zamanlayıcılar ve yazımlar (yeni komut çıkmaz),
  // sonra veri kaynakları (executor iptal + join); rclcpp::shutdown en son
  // Runtime yıkıcısında.
  backend.stop();
  rt.stop();
  std::fprintf(stdout, "forklift_gui: düzenli kapanış tamam\n");
  std::fflush(stdout);
  return rc;
}
