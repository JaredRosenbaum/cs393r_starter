
#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

#include <queue>

void resample (std::vector<particle_filter::Particle> &particles_);
void printParticles(const std::vector<particle_filter::Particle> &particles);

int main(int argc, char** argv)
{
    util_random::Random rng_;

    std::vector<particle_filter::Particle> particles;
    particle_filter::Particle particle;
    particle.weight = 1.0;
    particle.angle = 0.5;
    particles.push_back(particle);

    particle.weight = 0.0;
    particle.angle = -0.1;
    particles.push_back(particle);

    particle.weight = 1.5;
    particle.angle = -0.9;
    particles.push_back(particle);

    std::cout << "Starting with " << particles.size() << " particle(s)." << std::endl;
    resample(particles);
    std::cout << "Resampled to " << particles.size() << " particle(s)." << std::endl;
    printParticles(particles);

    for (int i = 0; i < 8; i++) {
        std::cout << "Starting with " << particles.size() << " particle(s)." << std::endl;
        for (auto &particle : particles) {
            if (particle.weight == 0) {
                particle.weight += 1;
            }
        }
        resample(particles);
        std::cout << "Resampled to " << particles.size() << " particle(s)." << std::endl;
        printParticles(particles);
    }

    return 0;
}

void resample (std::vector<particle_filter::Particle> &particles_)
{
    util_random::Random rng_;

    const int n_particles {5};
    std::vector<particle_filter::Particle> resampled_particles;
    resampled_particles.reserve(n_particles);

    if (particles_.empty()) {
        std::cout << "NO PARTICLES RECEIVED! Nothing to resample..." << std::endl;
        return;
    }

    // get max weight for normalization
    double max_particle_weight {particles_[0].weight};
    if (static_cast<int>(particles_.size()) > 1) {
        for (std::size_t i = 1; i < particles_.size(); i++) {
        if (particles_[i].weight > max_particle_weight) {
            max_particle_weight = particles_[i].weight;
        }
        }
    }

    // std::cout << "Max particle weight: " << max_particle_weight << std::endl;

    // normalize and build discrete "distribution" for resampling
    double weights_sum {0.d};
    std::queue<double> relative_positions;
    for (std::size_t i = 0; i < particles_.size(); i++) {
        // std::cout << "\tParticle weight adjusted from " << particles_[i].weight;
        particles_[i].weight -= max_particle_weight; // normalizing the particle's weight
        // std::cout << " to " << particles_[i].weight << std::endl;
        // std::cout << "\tWeights sum increased from " << weights_sum;
        weights_sum += exp(particles_[i].weight); // converting this out of log space to get the range for low-variance resampling
        // std::cout << " to " << weights_sum << std::endl;
        relative_positions.push(weights_sum); // keeping track of where the particles fall in the "distribution" we are going to resample
        // std::cout << "\tAdded a relative position at " << weights_sum << "\n" << std::endl;
    }

    // assert(n_particles == (int)(relative_positions.size()));

    // std::cout << "Number of relative positions: " << relative_positions.size() << std::endl;

    // create starting point for resampling
    // double r {rng_.UniformRandom(0, weights_sum / n_particles)};
    double low_variance_sampling_step {weights_sum / n_particles};
    double current_sampling_location {rng_.UniformRandom(0, low_variance_sampling_step)};

    // std::cout << "Sampling starting from " << current_sampling_location << " with step size " << low_variance_sampling_step << std::endl;

    // resample
    int original_particle_counter {0};
    for (std::size_t i = 0; i < n_particles; i++) {
        // std::cout << "\tResampling step " << i << std::endl;
        while (!relative_positions.empty()) {
            // std::cout << "\t\tEntering while..." << std::endl;
            // std::cout << "\t\t\tCurrent sampling location: " << current_sampling_location << ", front: " << relative_positions.front() << std::endl; 
            if (relative_positions.front() < current_sampling_location) {
                // std::cout << "\t\t\tPopping front!" << std::endl;
                relative_positions.pop();
                original_particle_counter++;
            }
            current_sampling_location += low_variance_sampling_step;
            // std::cout << "\t\t\tUpdating sampling location to " << current_sampling_location << std::endl;
            resampled_particles.push_back(particles_[original_particle_counter]);
            // std::cout << "\t\t\tAdded particle " << original_particle_counter << " from original list to output." << std::endl;
            break;
        }
    }

    // assert(n_particles == (int)(resampled_particles.size()));

    printParticles(resampled_particles);

    // !!! should all the particles be unweighted now? Should they be explicitly set to some value or just kept as is here?
    // double resampled_weight {log(1.d / n_particles)};
    // std::cout << "Weight: " << resampled_weight << std::endl; 
    // for (auto &particle : resampled_particles) {
    //     particle.weight = resampled_weight;
    // }

    // assign and return
    particles_ = resampled_particles;
}

void printParticles(const std::vector<particle_filter::Particle> &particles)
{
    for (const auto &particle : particles) {
        std::cout << "\tParticle {angle: " << particle.angle << ", weight: " << std::round(100 * particle.weight) / 100  << "}" << std::endl;
    }
}
