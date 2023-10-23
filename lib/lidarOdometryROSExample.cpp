#include "lidarOdomIO.h"
#include "lidarOdometry.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

class PointCloudIMUSubscriber : public rclcpp::Node {
public:
  PointCloudIMUSubscriber() : Node("pointcloud_imu_subscriber") {
    // Inicjalizacja subskrypcji PointClouda
    pointcloud_subscriber =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mid360", 1,
            std::bind(&PointCloudIMUSubscriber::pointCloudCallback, this,
                      std::placeholders::_1));

    // Inicjalizacja subskrypcji danych IMU
    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_topic", 1,
        std::bind(&PointCloudIMUSubscriber::imuDataCallback, this,
                  std::placeholders::_1));
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Konwersja sensor_msgs::msg::PointCloud2 na pcl::PointCloud
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc, pcl_cloud);

    // Przetwarzanie i przechowywanie punktów w wektorze Point3Di
    for (const pcl::PointXYZI &point : pcl_cloud.points) {
      Point3Di p;
      p.point << point.x, point.y, point.z;
      p.timestamp = point.data[3]; // Zakładam, że timestamp jest przechowywany
                                   // w czwartej współrzędnej
      p.intensity = 255; // Nie ma tego
      p.index_pose = 0;  // Ustaw indeks pozy według potrzeb
      pointcloud.push_back(p);
    }
  }

  std::vector<Point3Di> getPointCloud() { return pointcloud; }

  void imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
    FusionVector gyro, acc;
    gyro.axis.x = msg->angular_velocity.x;
    gyro.axis.y = msg->angular_velocity.y;
    gyro.axis.z = msg->angular_velocity.z;
    acc.axis.x = msg->linear_acceleration.x;
    acc.axis.y = msg->linear_acceleration.y;
    acc.axis.z = msg->linear_acceleration.z;
    std::tuple<double, FusionVector, FusionVector> imu_data_tuple(timestamp,
                                                                  gyro, acc);
    imu_data.push_back(imu_data_tuple);
    mandeye::SlamConfig config;
    config.tempSave = "/home/rabin/Documents/mandeye/test3";
    mandeye::optimizeTrajectory(imu_data, pointcloud, config);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  mandeye::InputPointCloudData pointcloud;
  mandeye::InputImuData imu_data;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto pc_imu_subscriber = std::make_shared<PointCloudIMUSubscriber>();

  rclcpp::spin(pc_imu_subscriber);
  rclcpp::shutdown();
  return 0;
}
