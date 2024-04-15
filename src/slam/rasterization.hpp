#pragma once

#include <vector>
#include <memory>
#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <random>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

namespace rasterization {

class LookupTable {
public:
    LookupTable(const std::vector<Eigen::Vector2f> &points, const float &resolution, const float &sigma)
    : _resolution(resolution), _sigma(sigma)
    {
        // create the lookup table based on the data
        generateLookupTable(points);
    }

    double lookupUnnormalizedProbability(const Eigen::Vector2f &point)
    {
        // calculate indices of point in image
        const auto indices {computeIndices(point)};
        if (indices[0] == -1 || indices[1] == -1) {return 0.d;}

        // if here, the point is within the canvas
        return (*_table)[indices[0]][indices[1]];
    }

    double getPeak()
    {
        return 1.d / sqrt(2 * M_PI * _sigma); 
    }

    void exportAsPPM(const std::string &path)
    {
        const auto pixels = *_table;
        std::ofstream outFile(path);

        // Write PPM header
        outFile << "P3\n"; // P3 indicates the type of PPM file (ASCII format)
        outFile << pixels[0].size() << " " << pixels.size() << "\n"; // Width and height
        outFile << "255\n"; // Maximum color value (for 8-bit grayscale)

        // find highest magnitude pixel
        int max_value {};
        for (const auto &row : pixels) {
            for (const auto &value : row) {
                if (value > max_value) {max_value = value;}
            }
        }

        // Write pixel data
        for (auto row : pixels) {
            for (auto value : row) {
                int int_value = static_cast<int>(255 * value / max_value);
                if (int_value > 255) {
                    int_value = 255;
                }
                outFile << int_value << " " << int_value << " " << int_value << "\n"; // Write grayscale pixel (R, G, B)
            }
        }
        outFile.close();
        std::cout << "Saved image to " << path << std::endl;
    }

    void exportAsPPMRandom(const std::string &path)
    {
        const auto pixels = *_table;
        std::ofstream outFile(path);

        // Write PPM header
        outFile << "P3\n"; // P3 indicates the type of PPM file (ASCII format)
        outFile << pixels[0].size() << " " << pixels.size() << "\n"; // Width and height
        outFile << "255\n"; // Maximum color value (for 8-bit grayscale)

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(0, 255);

        // Write pixel data
        for (auto row : pixels) {
            for (auto value : row) {
                if (value){}
                outFile << dis(gen) << " " << dis(gen) << " " << dis(gen) << "\n"; // Write grayscale pixel (R, G, B)
            }
        }
        outFile.close();
        std::cout << "Saved image to " << path << std::endl;
    }

private:
    const float _resolution;
    const float _sigma;
    std::unique_ptr<std::vector<std::vector<double>>> _table;
    std::unique_ptr<const Eigen::Vector2f> _lower_left_corner;
    std::unique_ptr<const Eigen::Vector2f> _upper_right_corner;

    std::array<int, 2> computeIndices(const Eigen::Vector2f &point)
    {
        // check if image not yet created (should never get here but just n case)
        if (_table == nullptr || _lower_left_corner == nullptr || _upper_right_corner == nullptr) {
            std::cout << "Image is not initialized! Nothing to look up." << std::endl;
            return {-1, -1};
        }

        // make sure the point falls within the bounds of the input data
        if (point.x() < _lower_left_corner->x() || point.x() > _upper_right_corner->x() || point.y() < _lower_left_corner->y() || point.y() > _upper_right_corner->y()) {
            return {-1, -1};
        }

        // calculate the indices where the data should be
        int x_index {static_cast<int>((point.x() - _lower_left_corner->x()) / _resolution)};
        int y_index {static_cast<int>((point.y() - _lower_left_corner->y()) / _resolution)};

        // make sure these indices fall within the canvas
        if (!((x_index < static_cast<int>(_table->size())) && (x_index >= 0)) || !((y_index < static_cast<int>(_table->at(0).size())) && (y_index >= 0))) {
            return {-1, -1};
        }

        return {x_index, y_index};
    }

    void generateLookupTable(const std::vector<Eigen::Vector2f> &points)
    {
        // - iterate over points, find min and max x, y
        float min_x, max_x, min_y, max_y;
        min_x = min_y = std::numeric_limits<float>::max();
        max_x = max_y = -1 * std::numeric_limits<float>::max();

        for (const auto &point : points) {
            if (point.x() < min_x) {min_x = point.x();}
            if (point.y() < min_y) {min_y = point.y();}
            if (point.x() > max_x) {max_x = point.x();}
            if (point.y() > max_y) {max_y = point.y();}
        }
        float buffer {1.f}; // m // setting to be 3*_sigma at minimum to avoid issues with indexing when seeding Gaussians
        _lower_left_corner = std::make_unique<const Eigen::Vector2f>(min_x - buffer, min_y - buffer);
        _upper_right_corner = std::make_unique<const Eigen::Vector2f>(max_x + buffer, max_y + buffer);

        // - now build an "image" sized for these limits
        std::size_t size_x {static_cast<std::size_t>((_upper_right_corner->x() - _lower_left_corner->x()) / _resolution)};
        std::size_t size_y {static_cast<std::size_t>((_upper_right_corner->y() - _lower_left_corner->y()) / _resolution)};

        _table = std::make_unique<std::vector<std::vector<double>>>(size_x, std::vector<double>(size_y, 0.d));

        std::cout << "Created an lookup table of size [" << _table->size() << ", " << _table->at(0).size() << "] with lower left corner [" << _lower_left_corner->transpose() << "] and upper right corner [" << _upper_right_corner->transpose() << "]." << std::endl;

        for (const auto &point : points) {
            seedGaussianKernel(point);
        }
    }

    void seedGaussianKernel(const Eigen::Vector2f &point)
    {
        const auto indices {computeIndices(point)};
        if (indices[0] == -1 || indices[1] == -1) {return;}

        // get all points that should have values
        float d_three_sigma {3 * _sigma};
        int d_pixels {static_cast<int>(d_three_sigma / _resolution)};

        // evaluate the Gaussian at these points (in the center) based on distance from point
        for (int i = std::max(indices[0] - d_pixels, 0); i <= std::min(indices[0] + d_pixels, static_cast<int>(_table->size()) - 1); i++) {
            
            for (int j = std::max(indices[1] - d_pixels, 0); j <= std::min(indices[1] + d_pixels, static_cast<int>((*_table)[0].size()) - 1); j++) {

                // calculate the distance from the center to the pixel in pixels
                double l2_pixels {sqrt(pow(i - indices[0], 2) + pow(j - indices[1], 2))};

                // check if it's within the circle
                if (l2_pixels <= d_pixels) {
                    
                    // if it is, convert the distance to m
                    float l2_m {static_cast<float>(l2_pixels * _resolution)};
                    
                    // and evaulate the Gaussian at this distance
                    auto prob {exp(-0.5 * pow(l2_m / _sigma, 2))};

                    // paper suggest this should be a max of all the possible probability values, is this right?
                    // (*_table)[i][j] += prob;
                    (*_table)[i][j] = std::max(prob, (*_table)[i][j]);
                }
            }
        }
    }

}; // class LookupTable

} // namespace rasterization
