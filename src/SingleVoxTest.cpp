#include <octomap/octomap.h> 
#include <octomap_msgs/Octomap.h> 
#include <octomap_msgs/conversions.h> 
#include <iostream> 
#include <pcl_conversions/pcl_conversions.h> 
#include <ros/ros.h> 
#include <unistd.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> RGBPointCloud;

RGBPointCloud::Ptr buildPC() {
  pcl::PointXYZRGB p;
  RGBPointCloud::Ptr pc (new RGBPointCloud);

  for(float i=-100; i<100; i++) {
    for(float j=-100; j<100; j++) {
     // for(float k=0; k<100; k++){
        p.x = i/30;
        p.y = j/30;
       // p.z = k/30;

        pc->points.push_back(p);
     // }
    }
  }

  return pc;
}

void publishPC() {
  ros::NodeHandle n;
  ros::Publisher pub;
  pub = n.advertise<sensor_msgs::PointCloud2>("testVox", 2);
  RGBPointCloud::Ptr pc = buildPC();
  sensor_msgs::PointCloud2 pc_msg;
  pcl::toROSMsg(*pc, pc_msg);
  pc_msg.header.frame_id = "map";
  ros::Rate loop(0.5);
  while(1) {
    pub.publish(pc_msg);
    loop.sleep();
  }
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "testVox");
  publishPC();
}
