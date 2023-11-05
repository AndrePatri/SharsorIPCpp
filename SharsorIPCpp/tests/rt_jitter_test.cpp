#include <gtest/gtest.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <sched.h>
#include <sys/mman.h>

constexpr long long NSEC_PER_SEC = 1000000000LL;
constexpr long long INTERVAL = NSEC_PER_SEC / 1000; // 1kHz

constexpr size_t ITERATIONS = 1000; 
constexpr double JITTER_THRESH = 50000.0; // Jitter threshold in nanoseconds 

void control_loop() {
    
    std::cout << "Control loop executed." << std::endl;

}

class RealTimeTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set low real-time priority
        sched_param param_low;
        param_low.sched_priority = sched_get_priority_min(SCHED_FIFO);
        if (sched_setscheduler(0, SCHED_FIFO, &param_low) == -1) {
            perror("sched_setscheduler failed (low priority)");
            FAIL();
        }

        // TODO: Any other low-priority setup logic goes here

        // Set high real-time priority for control loop
        sched_param param_high;
        param_high.sched_priority = sched_get_priority_max(SCHED_FIFO);
        if (sched_setscheduler(0, SCHED_FIFO, &param_high) == -1) {
            perror("sched_setscheduler failed (high priority)");
            FAIL();
        }

        // Pin process to the 4th core (index 3)
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(3, &cpuset);
        if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1) {
            perror("sched_setaffinity failed");
            FAIL();
        }

        // Lock memory
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
            perror("mlockall failed");
            FAIL();
        }
    }

    void TearDown() override {
        // Optionally, add any cleanup code here
    }
};

TEST_F(RealTimeTest, ControlLoopExecution) {
    
    std::array<double, ITERATIONS> jitter_measurements;

    // Control loop with sleep duration adjustment
    auto next = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < ITERATIONS; ++i) {
        
        auto expected_end = next + std::chrono::nanoseconds(INTERVAL);

        control_loop();

        auto actual_end = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(actual_end - next).count();
        jitter_measurements[i] = static_cast<double>(duration) - INTERVAL;

        if (duration < INTERVAL) {

            std::this_thread::sleep_for(std::chrono::nanoseconds(INTERVAL - duration));

        } else {

            std::cerr << "Control loop overrun at iteration " << i << "!" << std::endl;

        }

        next += std::chrono::nanoseconds(INTERVAL);
    }

    // Assert jitter constraints
    bool jitter_within_limits = std::all_of(jitter_measurements.begin(), 
                                            jitter_measurements.end(),
                                            [](double jitter) {

                                                return std::abs(jitter) <= JITTER_THRESH;

                                            } );

    ASSERT_TRUE(jitter_within_limits) << 
        "Jitter exceeded threshold in one or more iterations!";

}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}