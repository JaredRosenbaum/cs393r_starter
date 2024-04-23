#include "icp.h"
#include <iostream>
#include <fstream>

std::vector<float> readFloatFile(const std::string& filename) {
    std::vector<float> floats;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return floats; // Return empty vector if file cannot be opened
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')) {
            float value = std::stof(token);
            floats.push_back(value);
        }
    }
    
    file.close();
    return floats;
}

std::vector<Eigen::Vector2f> convertScan(std::vector<float> ranges) {
    // Convert scans to point clouds
    std::vector<Eigen::Vector2f> point_cloud;
    float angle_max = 2.356194496154785;
    float angle_min = -2.356194496154785;
    float range_max = 10;
    float angle_inc = (angle_max - angle_min) / ranges.size();
    for (std::size_t i = 0; i < ranges.size(); i++) {
        // Ignore points that are not obstacles
        if (ranges[i] >= range_max) {
            continue;
        }

        // Create point coordinates in robot frame
        const Eigen::Vector2f lidar_loc(0.2, 0);    // LiDAR is 20cm in front of base link
        float theta = angle_inc * i + angle_min;    // angle_min is -2.35619
        Eigen::Vector2f point(
            ranges[i] * cos(theta) + lidar_loc.x(),
            ranges[i] * sin(theta) + lidar_loc.y()
        );

        // Push into point cloud vector
        point_cloud.push_back(point);
    }

    return point_cloud;
}

int main (int argc, char** argv)
{
    // create a cloud (probably make it simple and geometric?)
    auto raw_data {readFloatFile("/home/dev/cs393r_starter/test_data/gtsam_poses_and_scans/scan1.txt")};
    
    auto target {convertScan(raw_data)}; 

    // transform it a bit
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity(3, 3);
    std::vector<Eigen::Vector2f> source {icp::transformPoints(target, T)};

    // compute registration
    bool s {icp::performICP(source, target, 100, 1e-3)};
    std::cout << "Success? " << s << std::endl;

    // time it

    return 0;
}
