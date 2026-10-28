// 本次实验专用：由 RViz 原生渲染器保存画面，避免 WSLg 的外部窗口截图返回黑图。
#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_rendering/render_window.hpp>
#include <iostream>
int main(int argc,char** argv) {
  if(argc<3)return 1;
  std::string config=argv[1],output=argv[2];
  rclcpp::init(argc,argv);
  QApplication app(argc,argv);
  auto node=std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_capture");
  rviz_common::VisualizationFrame frame(node);frame.setApp(&app);frame.setSplashPath("");
  frame.initialize(node,QString::fromStdString(config));frame.show();
  QTimer::singleShot(20000,[&](){
    frame.getRenderWindow()->captureScreenShot(output);
    std::cout<<"RViz native render saved: "<<output<<std::endl;
    rclcpp::shutdown();
    app.quit();
  });
  return app.exec();
}
