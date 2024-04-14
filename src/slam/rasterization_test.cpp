#include "rasterization.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char** argv)
{
    // generating 1081 random points (size of one scan)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-10, 10);
    std::uniform_real_distribution<float> dis2(-3, 3);

    std::vector<Eigen::Vector2f> points;
    for (int i = 0; i <= 1081; i++) {
        points.push_back(Eigen::Vector2f(dis(gen), dis2(gen)));
    }

    // creating coarse and fine lookup tables
    const float sigma {0.1f};

    // coarse table
    auto coarse_table_start {std::chrono::high_resolution_clock::now()};

    float coarse_resolution {0.3};
    const auto coarse_table {std::make_unique<rasterization::LookupTable>(points, coarse_resolution, sigma)};
    
    auto coarse_table_end {std::chrono::high_resolution_clock::now()};
    auto coarse_table_duration {std::chrono::duration_cast<std::chrono::microseconds>(coarse_table_end - coarse_table_start)};
    std::cout << "\t>> Created coarse lookup table in " << coarse_table_duration.count() << " us." << std::endl;

    // fine table
    auto fine_table_start {std::chrono::high_resolution_clock::now()};

    float fine_resolution {0.01}; // ! paper actually uses 0.03
    const auto fine_table {std::make_unique<rasterization::LookupTable>(points, fine_resolution, sigma)};

    auto fine_table_end {std::chrono::high_resolution_clock::now()};
    auto fine_table_duration {std::chrono::duration_cast<std::chrono::microseconds>(fine_table_end - fine_table_start)};
    std::cout << "\t>> Created fine lookup table in " << fine_table_duration.count() << " us." << std::endl;

    // saving images to make sure pixels look good
    coarse_table->saveAsPPM("/home/dev/cs393r_starter/images/pixels_coarse.ppm");
    coarse_table->saveAsPPMRandom("/home/dev/cs393r_starter/images/pixels_coarse_random.ppm");
    fine_table->saveAsPPM("/home/dev/cs393r_starter/images/pixels_fine.ppm");
    fine_table->saveAsPPMRandom("/home/dev/cs393r_starter/images/pixels_fine_random.ppm");

    // now perform lookups a lot of times for all points and see the efficiency
    int n_loops {static_cast<int>(1e6)};

    auto coarse_start_time {std::chrono::high_resolution_clock::now()};
    for (int i = 0; i < n_loops; i++) {
        float score {};
        for (const auto &point : points) {
            score += coarse_table->lookupProbability(point);
        }
    }
    auto coarse_end_time {std::chrono::high_resolution_clock::now()};
    auto coarse_duration {std::chrono::duration_cast<std::chrono::milliseconds>(coarse_end_time - coarse_start_time)};
    std::cout << "\t>> " << n_loops << " loops of coarse lookups finished in " << coarse_duration.count() << " ms.\n\t\tAverage time per loop: " << static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(coarse_duration).count()) / n_loops << " us." << std::endl;

    auto fine_start_time {std::chrono::high_resolution_clock::now()};
    for (int i = 0; i < n_loops; i++) {
        float score {};
        for (const auto &point : points) {
            score += fine_table->lookupProbability(point);
        }
    }
    auto fine_end_time {std::chrono::high_resolution_clock::now()};
    auto fine_duration {std::chrono::duration_cast<std::chrono::milliseconds>(fine_end_time - fine_start_time)};
    std::cout << "\t>> " << n_loops << " loops of fine lookups finished in " << fine_duration.count() << " ms.\n\t\tAverage time per loop: " << static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(fine_duration).count()) / n_loops << " us." <<  std::endl;

    // checking to make sure coarse and fine tables decay similarly as the "scan" is further perturbed from where it actually is
    for (int i = 0; i < 1000; i++) {
        float coarse_score {};
        float fine_score {};
        for (auto &point : points) {
            point.y() += 0.01;
            auto prob {coarse_table->lookupProbability(point)};
            coarse_score += prob;
        }
        for (const auto &point : points) {
            // point.y() += 0.01;
            auto prob {fine_table->lookupProbability(point)};
            fine_score += prob;
        }
        if (i % 10 == 0) {
            std::cout << coarse_score << ", " << fine_score << std::endl;
        }
    }

    std::cout << "FIN" << std::endl;
    return 0;
}
