#include "rasterization.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char** argv)
{
    // generating 1081 random points (size of one scan)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-10, 10);
    std::uniform_real_distribution<float> dis2(-10, 10);

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

    float fine_resolution {0.03}; // ! paper actually uses 0.03
    const auto fine_table {std::make_unique<rasterization::LookupTable>(points, fine_resolution, sigma)};

    auto fine_table_end {std::chrono::high_resolution_clock::now()};
    auto fine_table_duration {std::chrono::duration_cast<std::chrono::microseconds>(fine_table_end - fine_table_start)};
    std::cout << "\t>> Created fine lookup table in " << fine_table_duration.count() << " us." << std::endl;

    // saving images to make sure pixels look good
    coarse_table->exportAsPPM("/home/dev/cs393r_starter/images/pixels_coarse.ppm");
    coarse_table->exportAsPPMRandom("/home/dev/cs393r_starter/images/pixels_coarse_random.ppm");
    fine_table->exportAsPPM("/home/dev/cs393r_starter/images/pixels_fine.ppm");
    fine_table->exportAsPPMRandom("/home/dev/cs393r_starter/images/pixels_fine_random.ppm");

    float score {};
    for (const auto &point : points) {
        score += fine_table->evaluate(point);
    }
    // score /= (points.size() * fine_table->getPeak());
    score /= (points.size());
    std::cout << score << std::endl;

    // now perform lookups a lot of times for all points and see the efficiency
    int n_loops {static_cast<int>(1e6)};

    auto coarse_start_time {std::chrono::high_resolution_clock::now()};
    for (int i = 0; i < n_loops; i++) {
        float score {};
        for (const auto &point : points) {
            score += coarse_table->evaluate(point);
        }
    }
    auto coarse_end_time {std::chrono::high_resolution_clock::now()};
    auto coarse_duration {std::chrono::duration_cast<std::chrono::milliseconds>(coarse_end_time - coarse_start_time)};
    std::cout << "\t>> " << n_loops << " loops of coarse lookups finished in " << coarse_duration.count() << " ms.\n\t\tAverage time per loop: " << static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(coarse_duration).count()) / n_loops << " us." << std::endl;

    auto fine_start_time {std::chrono::high_resolution_clock::now()};
    for (int i = 0; i < n_loops; i++) {
        float score {};
        for (const auto &point : points) {
            score += fine_table->evaluate(point);
        }
    }
    auto fine_end_time {std::chrono::high_resolution_clock::now()};
    auto fine_duration {std::chrono::duration_cast<std::chrono::milliseconds>(fine_end_time - fine_start_time)};
    std::cout << "\t>> " << n_loops << " loops of fine lookups finished in " << fine_duration.count() << " ms.\n\t\tAverage time per loop: " << static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(fine_duration).count()) / n_loops << " us." <<  std::endl;

    // checking to make sure coarse and fine tables decay similarly as the "scan" is further perturbed from where it actually is
    for (int i = 0; i < 1000; i++) {
        float coarse_score {};
        float fine_score {};

        for (const auto &point : points) {
            auto prob {coarse_table->evaluate(point)};
            coarse_score += prob;
        }
        // coarse_score /= (coarse_table->getPeak() * points.size());
        coarse_score /= (points.size());

        for (auto &point : points) {
            auto prob {fine_table->evaluate(point)};
            fine_score += prob;
            point.y() += 0.01;
        }
        // fine_score /= (fine_table->getPeak() * points.size());
        fine_score /= (points.size());

        if (i % 10 == 0) {
            std::cout << coarse_score << ", " << fine_score << std::endl;
        }
    }

    std::cout << "FIN" << std::endl;
    return 0;
}
