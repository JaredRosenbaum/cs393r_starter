#include "rasterization.hpp"

using namespace rasterization;

int main(int argc, char** argv)
{
    // std::vector<Eigen::Vector2f> points;
    // for (int i = -10; i <= 10; i++) {
    //     points.push_back(Eigen::Vector2f(i, i));
    // }

    // const float sigma {0.5f};

    // float coarse_resolution {0.3};
    // auto coarse_table {LookupTable(points, coarse_resolution, sigma)};

    // float fine_resolution {0.03};
    // auto fine_table {LookupTable(points, fine_resolution, sigma)};

    // // TODO create test cases for boundaries
    // // std::cout << fine_table.lookupProbability(Eigen::Vector2f(0, 4)) << std::endl;

    // // TODO create test cases for gaussian

    // for (const auto &point : points) {
    //     fine_table.seedGaussianKernel(point);
    // }
    // auto gaussian_center {Eigen::Vector2f(0, 0)};
    // fine_table.seedGaussianKernel(gaussian_center);

    // double max_prob {fine_table.lookupProbability(gaussian_center)};
    
    // for (int i = 1; i < 100; i++) {
        
    //     double new_prob {fine_table.lookupProbability(Eigen::Vector2f(gaussian_center.x() + i * 0.01, gaussian_center.y() + i * 0.01))};

    //     // std::cout << new_prob << ", " << max_prob << std::endl;

    //     if (!(new_prob <= max_prob)) {
    //         std::cout << "!!!" << std::endl;
    //     }
    //     max_prob = new_prob;
    // }

    // fine_table.saveAsPPM();
    // fine_table.saveAsPPMRandom();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-10, 10);
    std::uniform_real_distribution<float> dis2(-3, 3);

    // generating 1081 random points (size of one scan)
    std::vector<Eigen::Vector2f> points;
    for (int i = 0; i <= 1081; i++) {
        points.push_back(Eigen::Vector2f(dis(gen), dis2(gen)));
    }

    // std::vector<Eigen::Vector2f> points;
    // for (int i = 0; i <= 10; i++) {
    //     points.push_back(Eigen::Vector2f(dis(gen), dis2(gen)));
    // }

    // creating coarse and fine lookup tables
    const float sigma {0.1f};

    float coarse_resolution {0.1};
    auto coarse_table {LookupTable(points, coarse_resolution, sigma)};

    float fine_resolution {0.01};
    auto fine_table {LookupTable(points, fine_resolution, sigma)};

    // saving images to make sure pixels look good
    coarse_table.saveAsPPM("/home/dev/cs393r_starter/pixels_coarse.ppm");
    coarse_table.saveAsPPMRandom("/home/dev/cs393r_starter/pixels_coarse_random.ppm");
    fine_table.saveAsPPM("/home/dev/cs393r_starter/pixels_fine.ppm");
    fine_table.saveAsPPMRandom("/home/dev/cs393r_starter/pixels_fine_random.ppm");

    // making sure score decays as we get further away from the truth
    for (int i = 0; i < 1000; i++) {
        float coarse_score {};
        float fine_score {};
        for (auto &point : points) {
            point.y() += 0.01;
            auto prob {coarse_table.lookupProbability(point)};
            coarse_score += prob;
        }
        for (auto &point : points) {
            // point.y() += 0.01;
            auto prob {fine_table.lookupProbability(point)};
            fine_score += prob;
        }
        std::cout << coarse_score << ", " << fine_score << std::endl;
    }

    std::cout << "FIN" << std::endl;
    return 0;
}
