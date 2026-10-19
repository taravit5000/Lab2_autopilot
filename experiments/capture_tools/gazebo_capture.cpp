#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/MainWindow.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/Events.hh>
#include <QTimer>
#include <mutex>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cmath>

class LabCapture : public gazebo::GUIPlugin
{
  gazebo::event::ConnectionPtr post_render_;
  std::mutex pending_mutex_;
  std::string pending_path_;
  unsigned int pending_frames_ = 0;
  void requestFrame(const std::string &path)
  {
    std::lock_guard<std::mutex> guard(pending_mutex_);
    pending_path_ = path;
    pending_frames_ = 0;
  }
  void saveAfterRender()
  {
    std::string path;
    {
      std::lock_guard<std::mutex> guard(pending_mutex_);
      if (pending_path_.empty() || ++pending_frames_ < 3) return;
      path.swap(pending_path_);
    }
    auto camera = gazebo::gui::get_active_camera();
    if (camera) std::cout << "Gazebo rendered frame saved: " << camera->SaveFrame(path) << " " << path << std::endl;
  }
  void setView(gazebo::rendering::UserCameraPtr camera, int phase)
  {
    const char *raw = std::getenv("LAB2_VIEW_INDEX");
    const int index = raw ? std::atoi(raw) : 0;
    // Fixed zoom; vary azimuth.
    const double azimuth = -.90 + (index % 5) * .10 + (phase-1)*.10;
    const double radius = 6.0, height = 10.5;
    const double focus_x = 0, focus_y = 0;
    const double x = focus_x + radius * std::cos(azimuth), y = focus_y + radius * std::sin(azimuth);
    camera->SetWorldPose(ignition::math::Pose3d(x,y,height,0,std::atan2(height,radius),azimuth+M_PI));
  }
public:
  void Load(sdf::ElementPtr) override
  {
    // Hide the unused plugin panel.
    post_render_ = gazebo::event::Events::ConnectPostRender(std::bind(&LabCapture::saveAfterRender,this));
    setFixedSize(1,1);
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_TranslucentBackground);
    setStyleSheet("background: transparent; border: none;");
    hide();
    auto requests = new QTimer(this);
    connect(requests, &QTimer::timeout, [this]() {
      hide();
      const char *request = std::getenv("LAB2_GAZEBO_REQUEST");
      if (!request) return;
      std::ifstream stream(request); std::string path;
      if (std::getline(stream,path) && !path.empty()) {
        auto camera = gazebo::gui::get_active_camera();
        if (camera) {
          const int phase = path.find("_Mid") != std::string::npos ? 1 : path.find("_End") != std::string::npos ? 2 : 0;
          setView(camera,phase);
          // Wait for the new view to render.
          std::remove(request);
          QTimer::singleShot(500, [this,path]() { requestFrame(path); });
        }
      }
    });
    requests->start(250);
    QTimer::singleShot(2000, []() { auto window=gazebo::gui::get_main_window(); if(window) { window->showNormal(); window->resize(960,850); window->move(0,35); } });
    QTimer::singleShot(4500, [this]() {
      auto camera = gazebo::gui::get_active_camera();
      if (!camera) { std::cerr << "No Gazebo GUI camera\n"; return; }
      setView(camera,0);
      camera->SetCaptureData(true);
      QTimer::singleShot(1000, [this]() {
        const char *out = std::getenv("LAB2_GAZEBO_CAPTURE");
        if (out) requestFrame(out);
      });
    });
  }
};
GZ_REGISTER_GUI_PLUGIN(LabCapture)
