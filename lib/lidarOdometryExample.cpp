#include <iostream>
#include "lidarOdomIO.h"
#include "lidarOdometry.h"
#include <vector>
#include <string>
int main()
{
	std::cout << "Hello" << std::endl;

	std::vector<std::string> imu_files{
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0000.csv",
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0001.csv",
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0002.csv",
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0003.csv",
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0004.csv",
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0005.csv",
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0006.csv",
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0007.csv",
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0008.csv",
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0009.csv",
		"/home/rabin/Documents/mandeye/continousScanning_0005/imu0010.csv",
	};

	std::vector<std::string> pc_files{
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0000.laz",
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0001.laz",
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0002.laz",
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0003.laz",
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0004.laz",
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0005.laz",
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0006.laz",
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0007.laz",
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0008.laz",
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0009.laz",
		"/home/rabin/Documents/mandeye/continousScanning_0005/lidar0010.laz",
	};

	std::vector<std::tuple<double, FusionVector, FusionVector>> imu_data;
	std::vector<Point3Di> pointcloud;
	for (const auto file : imu_files)
	{
		auto data = mandeye::utilsIO::load_imu(file);
		imu_data.insert(imu_data.end(), std::make_move_iterator(data.begin()), std::make_move_iterator(data.end()));
	}
	for (const auto file : pc_files)
	{
		auto data = mandeye::utilsIO::load_point_cloud(file);
		pointcloud.insert(pointcloud.end(), std::make_move_iterator(data.begin()), std::make_move_iterator(data.end()));
	}

	mandeye::SlamConfig config;
	config.tempSave = "/home/rabin/Documents/mandeye/test3";
	mandeye::optimizeTrajectory(imu_data, pointcloud, config);
}