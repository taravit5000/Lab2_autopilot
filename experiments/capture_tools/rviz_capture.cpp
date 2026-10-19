#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_rendering/render_window.hpp>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <cstdio>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz2");
  node->get_raw_node()->set_parameter(rclcpp::Parameter("use_sim_time", true));
  rviz_common::VisualizationFrame frame(node);
  frame.setApp(&app);
  frame.initialize(node, QString::fromLocal8Bit(std::getenv("LAB2_RVIZ_CONFIG")));
  frame.setWindowTitle("RViz — Lab 2 SLAM map");
  frame.resize(1200,850);
  frame.show();
  QTimer::singleShot(8000, [&frame]() {
    const char *out = std::getenv("LAB2_RVIZ_CAPTURE");
    if (out) {
      frame.getRenderWindow()->captureScreenShot(out);
      std::cout << "RViz render-window screenshot saved: " << out << std::endl;
      frame.grab().save(QString::fromLocal8Bit(out) + ".window.png");
    }
  });
  QTimer requests;
  QObject::connect(&requests, &QTimer::timeout, [&frame]() {
    const char *request = std::getenv("LAB2_CAPTURE_REQUEST");
    if (!request) return;
    std::ifstream stream(request);
    std::string path,mode;
    if (std::getline(stream, path) && !path.empty()) {
      std::getline(stream,mode);
      if (mode == "detail") {
        auto view = frame.getManager()->getViewManager()->getCurrent();
        view->subProp("Scale")->setValue(140.0);
      }
      std::remove(request);
      QTimer::singleShot(500, [&frame,path]() {
        frame.getRenderWindow()->captureScreenShot(path);
        std::cout << "RViz screenshot: " << path << std::endl;
      });
    }
  });
  requests.start(250);
  QTimer shutdown;
  QObject::connect(&shutdown, &QTimer::timeout, [&app]() { if (!rclcpp::ok()) app.quit(); });
  shutdown.start(100);
  return app.exec();
}
