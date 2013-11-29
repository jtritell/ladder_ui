#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/default_plugin/interactive_marker_display.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/config.h"
#include "rviz/yaml_config_reader.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <teleop_msgs/RateControl.h>
#include <ladder_shaper/LadderState.h>

#include "fitter_interface.h"

void FitterInterface::updateParameters(const ladder_shaper::LadderState::ConstPtr& msg){
    //msg->data;
    int angle_degrees = msg->direction/M_PI * 180;
    QString str = QString("X position : %1\nY position : %2\nDirection : %3\nInclination : %4\n"
            "Width : %5\nSpacing : %6\nInitial Spacing : %7\nRail Height : %8\nWalkway Rail Height : %9") \
            .arg(msg->xpos).arg(msg->ypos).arg(angle_degrees) \
            .arg(msg->inclination).arg(msg->legs_width).arg(msg->spacing).arg(msg->spacing_first) \
            .arg(msg->rail_height).arg(msg->walkway_rail_height);
    ui->txt_parameters->setPlainText(str);
}

FitterInterface::FitterInterface(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui_fitterinterface)
{
    ui->setupUi(this);
    rviz::RenderPanel*render_panel;
    render_panel = new rviz::RenderPanel();
    render_panel->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    ui->verticalLayout->addWidget(render_panel);
    manager_ = new rviz::VisualizationManager( render_panel );
    render_panel->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();
    
    //load the configuration
    rviz::YamlConfigReader reader;
    rviz::Config config;
    reader.readFile( config, "/home/jordan/.rviz/robot_view.rviz");
    //  reader.readFile( config, "front_view.rviz");
    if(reader.error())
      std::cout<<"ERROR reading config\n";
    else{
      config = config.mapGetChild("Visualization Manager");
      manager_->load(config);
    }
    n_ = ros::NodeHandle();
    str_pub_ =n_.advertise<std_msgs::String>("export",1000);
    int_pub_ =n_.advertise<std_msgs::Int32>("reset",1000);
    int_pub2_ =n_.advertise<std_msgs::Int32>("rungs",1000);
    pcd_client_ =n_.serviceClient<teleop_msgs::RateControl>("relay");
    config_sub_ = n_.subscribe("/ladder_state",1000,&FitterInterface::updateParameters,this);
}

FitterInterface::~FitterInterface()
{
    delete ui;
}
void FitterInterface::toPlanner()
{
  std_msgs::String myMsg;
  myMsg.data = "Export";
  str_pub_.publish(myMsg);
  ros::spinOnce();
}

void FitterInterface::requestPointCloud()
{
    ui->lbl_pcd_status->setText("Requesting...");
    ui->lbl_pcd_status->setStyleSheet("QLabel { background-color : yellow;border-style : outset; border-width : 1px; border-color : black;}");
    teleop_msgs::RateControl srv;
    srv.request.Rate = -1.0;
    if (pcd_client_.call(srv))
    {
        ui->lbl_pcd_status->setText("Success");
        ui->lbl_pcd_status->setStyleSheet("QLabel { background-color : green;border-style : outset; border-width : 1px; border-color : black;}");
      }
    else
    {
        ui->lbl_pcd_status->setText("Service Failed");
        ui->lbl_pcd_status->setStyleSheet("QLabel { background-color : red;border-style : outset; border-width : 1px; border-color : black;}");
    }
    //if(pcd_pub.call)
}

void FitterInterface::resetLadder()
{
    std_msgs::Int32 myMsg;
    myMsg.data = int(ui->num_rungs->value());
    int_pub_.publish(myMsg);
    ros::spinOnce();
}

void FitterInterface::changeNumRungs(int num)
{
  rungs=num;
  std_msgs::Int32 myMsg;
  myMsg.data = rungs;
  int_pub2_.publish(myMsg);
  ros::spinOnce();
}
