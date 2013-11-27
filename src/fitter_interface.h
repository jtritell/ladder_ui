#ifndef FITTERINTERFACE_H
#define FITTERINTERFACE_H

#include <QWidget>
#include <ros/ros.h>
#include <QMainWindow>
#include "ui_LadderInterface.h"
#include <ladder_shaper/LadderState.h>


namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

namespace Ui {
class fitterinterface;
}

class FitterInterface : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit FitterInterface(QWidget *parent = 0);
    void updateParameters(const ladder_shaper::LadderState::ConstPtr& msg);
    ~FitterInterface();
    
private Q_SLOTS:
  void toPlanner();
  void requestPointCloud();
  void resetLadder();
  void changeNumRungs(int num);


private:
  rviz::VisualizationManager* manager_;
  Ui_fitterinterface *ui;
  ros::NodeHandle n_;
  ros::Publisher str_pub_;
  ros::Publisher int_pub_;
  ros::Publisher int_pub2_;
  ros::ServiceClient pcd_client_;
  ros::Subscriber config_sub_;
  int rungs;
  QLabel* pointcloud_status;
};

#endif // FITTERINTERFACE_H
