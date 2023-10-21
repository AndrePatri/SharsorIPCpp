// jitter_test.cpp

#include <gtest/gtest.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

const long PERIOD_US = 1000;  // Control loop period in microseconds
const int NUM_ITERATIONS = 10000;  // Number of iterations

long jitterArray[NUM_ITERATIONS];  // Pre-allocated array to store jitter values

void *controlLoop(void *arg) {
    struct timespec next_wakeup, actual_wakeup;
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

    for (int i = 0; i < NUM_ITERATIONS; i++) {
        next_wakeup.tv_nsec += PERIOD_US * 1000;
        while (next_wakeup.tv_nsec >= 1000000000) {
            next_wakeup.tv_nsec -= 1000000000;
            next_wakeup.tv_sec += 1;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);
        clock_gettime(CLOCK_MONOTONIC, &actual_wakeup);

        jitterArray[i] = (actual_wakeup.tv_sec - next_wakeup.tv_sec) * 1000000000 + (actual_wakeup.tv_nsec - next_wakeup.tv_nsec);
    }
    return NULL;
}

std::pair<double, long> getJitterValues() {
    pthread_t thread;
    pthread_create(&thread, NULL, controlLoop, NULL);
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(thread, SCHED_FIFO, &param);
    pthread_join(thread, NULL);

    long sumJitter = 0;
    long maxJitter = jitterArray[0];
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        sumJitter += jitterArray[i];
        if (jitterArray[i] > maxJitter) {
            maxJitter = jitterArray[i];
        }
    }
    double avgJitter = static_cast<double>(sumJitter) / NUM_ITERATIONS;

    return {avgJitter, maxJitter};
}

TEST(JitterTest, JitterBounds) {
    auto [avgJitter, maxJitter] = getJitterValues();

    // Here, you can make assertions based on your jitter requirements
    EXPECT_LT(avgJitter, 100);  // Example: Average jitter should be less than 100 ns
    EXPECT_LT(maxJitter, 500);  // Example: Max jitter should be less than 500 ns
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
