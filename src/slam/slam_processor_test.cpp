#include "slam_processor.h"
#include <chrono>

using namespace slam_processing;

int main(int argc, char** argv)
{
    // create points and odometry
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-10, 10);
    std::uniform_real_distribution<float> dis2(-10, 10);

    std::vector<Eigen::Vector2f> points;
    for (int i = 0; i <= 1081; i++) {
        points.push_back(Eigen::Vector2f(dis(gen), dis2(gen)));
    }
    Pose odom(0, 0, 0);
    auto start_time {std::chrono::high_resolution_clock::now()};

    // create a new processor
    auto processor {std::make_unique<Processor>(1)};

    processor->update(odom, points);

    for (auto &point : points) {
        point.x() += 1.f;
    }
    Pose updated_pose(1, 0, 0.1);
    processor->update(updated_pose, points);

    for (auto &point : points) {
        point.x() += 0.5f;
        point.y() += 0.5f;
    }
    Pose updated_pose_1(0.5, 0.4, 0.15);
    processor->update(updated_pose_1, points);
    

    // timing stuff
    auto end_time {std::chrono::high_resolution_clock::now()};
    auto time_duration {std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)};
    std::cout << time_duration.count() << "us" << std::endl;
    return 0;
}
