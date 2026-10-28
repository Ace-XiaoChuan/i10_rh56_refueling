// 实验辅助工具：保存 RViz 原生画面，避开 WSLg 外部截图返回黑图的问题。
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
    rclcpp::shutdown();app.quit();
  });
  return app.exec();
}
