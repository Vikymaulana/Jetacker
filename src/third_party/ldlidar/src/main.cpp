/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-10
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "lipkg.h"
#include "ros_api.h"

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, ldlidar::LiPkg* commpkg, LaserScanSetting& setting, ros::Publisher& lidarpub);

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldlidar_publiser");
  ros::NodeHandle nh; // create a ROS Node
  ros::NodeHandle nh_private("~");

  std::string product_name;
	std::string topic_name;
	std::string port_name;
  LaserScanSetting setting;

  nh_private.getParam("product_name", product_name);
	nh_private.param("topic_name", topic_name, std::string("scan"));
	nh_private.param("frame_id", setting.frame_id, std::string("base_laser"));
  nh_private.param("port_name", port_name, std::string("/dev/ttyUSB0"));
  nh_private.param("laser_scan_dir", setting.laser_scan_dir, bool(true));
  nh_private.param("enable_angle_crop_func", setting.enable_angle_crop_func, bool(false));
  nh_private.param("angle_crop_min", setting.angle_crop_min, double(0.0));
  nh_private.param("angle_crop_max", setting.angle_crop_max, double(0.0));

  
  ROS_INFO(" [ldrobot] <product_name>: %s,<topic_name>: %s,<port_name>: %s,<frame_id>: %s", 
    product_name.c_str(), topic_name.c_str(), port_name.c_str(), setting.frame_id.c_str());

  ROS_INFO(" [ldrobot] <laser_scan_dir>: %s,<enable_angle_crop_func>: %s,<angle_crop_min>: %f,<angle_crop_max>: %f",
    (setting.laser_scan_dir?"Counterclockwise":"Clockwise"), (setting.enable_angle_crop_func?"true":"false"), setting.angle_crop_min, setting.angle_crop_max);

  ldlidar::LiPkg *lidar_pkg = new ldlidar::LiPkg();
  ldlidar::CmdInterfaceLinux *cmd_port = new ldlidar::CmdInterfaceLinux();
  uint32_t baudrate = 0;
  ROS_INFO_STREAM(" [ldrobot] SDK Pack Version is " << lidar_pkg->GetSdkVersionNumber());
  if(product_name == "LDLiDAR_LD14") {
    baudrate = 115200;
    lidar_pkg->SetProductType(ldlidar::LDType::LD_14);
    lidar_pkg->SetLaserScanDir(setting.laser_scan_dir);
  } else if (product_name == "LDLiDAR_LD14P") {
    baudrate = 230400;
    lidar_pkg->SetProductType(ldlidar::LDType::LD_14P);
    lidar_pkg->SetLaserScanDir(setting.laser_scan_dir);
  } else if (product_name == "LDLiDAR_LD00") {
    baudrate = 115200;
    lidar_pkg->SetProductType(ldlidar::LDType::LD_00);
    lidar_pkg->SetLaserScanDir(setting.laser_scan_dir);
  } else if (product_name == "LDLiDAR_LD03") {
    baudrate = 115200;
    lidar_pkg->SetProductType(ldlidar::LDType::LD_03);
    lidar_pkg->SetLaserScanDir(setting.laser_scan_dir);
  } else if (product_name == "LDLiDAR_LD08") {
    baudrate = 115200;
    lidar_pkg->SetProductType(ldlidar::LDType::LD_08);
    lidar_pkg->SetLaserScanDir(setting.laser_scan_dir);
  } else{
    ROS_ERROR(" [ldrobot] Error, input param <product_name> is fail!!");
    exit(EXIT_FAILURE);
  }

  if (topic_name.empty()) {
    ROS_ERROR(" [ldrobot] fail, topic_name is empty!");
    exit(EXIT_FAILURE);
  }
  
  if (port_name.empty()) {
    ROS_ERROR(" [ldrobot] fail, input param <port_name> is empty!");
    exit(EXIT_FAILURE);
  }
  
  cmd_port->SetReadCallback(std::bind(&ldlidar::LiPkg::CommReadCallback, lidar_pkg, std::placeholders::_1, std::placeholders::_2));

  if (cmd_port->Open(port_name, baudrate)) {
    ROS_INFO(" [ldrobot] open device %s success", port_name.c_str());
  } else {
    ROS_ERROR(" [ldrobot] open device %s fail", port_name.c_str());
    exit(EXIT_FAILURE);
  }

  ros::Publisher lidar_pub = 
    nh.advertise<sensor_msgs::LaserScan>(topic_name, 10); // create a ROS topic 

  ros::Rate r(6); //Hz
  auto last_time = std::chrono::steady_clock::now();
  ROS_INFO("[ldrobot] pub lidar data");
  
  while (ros::ok()) {
    if (lidar_pkg->IsFrameReady()) {
      lidar_pkg->ResetFrameReady();
      last_time = std::chrono::steady_clock::now();
      ldlidar::Points2D laserscandata = lidar_pkg->GetLaserScanData();
      ToLaserscanMessagePublish(laserscandata, lidar_pkg, setting, lidar_pub);
    }

    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-last_time).count() > 1000) { 
			ROS_ERROR("[ldrobot] lidar pub data is time out, please check lidar device");
			exit(EXIT_FAILURE);
		}

    r.sleep();
  }

  cmd_port->Close();

  delete lidar_pkg;
  lidar_pkg = nullptr;
  delete cmd_port;
  cmd_port = nullptr;
  
  return 0;
}


void  ToLaserscanMessagePublish(ldlidar::Points2D& src, ldlidar::LiPkg* commpkg, LaserScanSetting& setting, ros::Publisher& lidarpub) {
  float angle_min, angle_max, range_min, range_max, angle_increment;
  float scan_time;
  ros::Time start_scan_time;
  static ros::Time end_scan_time;
  static bool first_scan = true;

  start_scan_time = ros::Time::now();
  scan_time = (start_scan_time - end_scan_time).toSec();

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }

  // Adjust the parameters according to the demand
  angle_min = 0;
  angle_max = (2 * M_PI);
  range_min = 0.02;
  range_max = 12;
  int beam_size = static_cast<int>(src.size());
  angle_increment = (angle_max - angle_min) / (float)(beam_size -1);

  // Calculate the number of scanning points
  if (commpkg->GetSpeedOrigin() > 0) {
    sensor_msgs::LaserScan output;
    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    if (beam_size <= 1) {
      output.time_increment = 0;
    } else {
      output.time_increment = scan_time / (beam_size - 1);
    }
    output.scan_time = scan_time;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity 
      float dir_angle = point.angle;;

      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN(); 
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      int index = (int)((angle - angle_min) / angle_increment);
      if (index < beam_size) {
        if (index < 0) {
          ROS_ERROR("[ldrobot] error index: %d, beam_size: %d, angle: %f, angle_min: %f, angle_increment: %f", 
                    index, beam_size, angle, angle_min, angle_increment);
        }
        // If the current content is Nan, it is assigned directly
        if (std::isnan(output.ranges[index])) {
          output.ranges[index] = range;
        } else { // Otherwise, only when the distance is less than the current
                //   value, it can be re assigned
          if (range < output.ranges[index]) {
            output.ranges[index] = range;
          }
        }
        output.intensities[index] = intensity;
      }
    }
    lidarpub.publish(output);
    end_scan_time = start_scan_time;
  } 
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/