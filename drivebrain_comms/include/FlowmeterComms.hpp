#pragma once
#include <linux/gpio.h>
#include <string>
#include <thread>
#include <atomic>
#include <DriverBus.hpp>
#include <cstdint>

namespace comms {
    class FlowmeterComms {
        public:
            FlowmeterComms(const std::string &gpio_chip, unsigned int gpio_line);
            std::uint64_t get_pulse_count() const;
            ~FlowmeterComms();
        
        private:
            void read_pulse();
            void process_pulse();

            core::ThreadSafeDeque<gpio_v2_line_event> pulse_queue_;

            std::atomic<bool> _running = false;
            std::atomic<std::uint64_t> pulse_count_ = 0;

            std::thread reader_thread;
            std::thread processing_thread;

            const std::string gpio_chip_;
            unsigned int gpio_line_;

            int chip_fd_ = -1;
            int line_fd_ = -1;
    };
}
