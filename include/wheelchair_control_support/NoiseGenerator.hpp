#ifndef WHEELCHAIR_CONTROL_SUPPORT_NOISE_GENERATOR_HPP
#define WHEELCHAIR_CONTROL_SUPPORT_NOISE_GENERATOR_HPP

#include <iostream>
#include <random>
#include <chrono>
#include <thread>
#include <utility>

class NoiseGenerator
{
public:
    // Constructor: Initializes the generator with frequency (Hz), mean, and stddev
    NoiseGenerator(double frequency, double mean, double stddev)
        : mean_(mean), stddev_(stddev), freq_(frequency)
    {
        // Initialize two independent random engines with different seeds
        std::random_device rd;
        gen1_ = std::mt19937(rd());
        gen2_ = std::mt19937(rd());

        dist1_ = std::normal_distribution<double>(mean_, stddev_);
        dist2_ = std::normal_distribution<double>(mean_, stddev_);

        interval_ = std::chrono::milliseconds(static_cast<int>(1000.0 / freq_));

        is_running_ = true;
        generator_thread_ = std::thread(&NoiseGenerator::generateRandomValues, this);
    }

    // Deleted default constructor
    NoiseGenerator() = delete;

    // Destructor
    ~NoiseGenerator()
    {
        is_running_ = false;
        if (generator_thread_.joinable()) {
            generator_thread_.join();
        }
    }

    // Getter for both noise values as a pair
    std::pair<double, double> getRandomValues() const
    {
        return {last_value1_, last_value2_};
    }

private:
    void generateRandomValues()
    {
        while (is_running_)
        {
            last_value1_ = dist1_(gen1_);
            last_value2_ = dist2_(gen2_);
            std::this_thread::sleep_for(interval_);
        }
    }

    double mean_;
    double stddev_;
    double freq_;
    std::chrono::milliseconds interval_;

    std::mt19937 gen1_, gen2_;
    std::normal_distribution<double> dist1_, dist2_;

    std::thread generator_thread_;
    bool is_running_;

    double last_value1_ = 0.0;
    double last_value2_ = 0.0;
};

#endif  // WHEELCHAIR_CONTROL_SUPPORT_NOISE_GENERATOR_HPP