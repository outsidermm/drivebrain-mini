#pragma once
#include <linux/gpio.h>
#include <string>
#include <thread>
#include <atomic>

namespace comms {
    class FlowmeterComms {
        public:
            FlowmeterComms(const std::string &gpio_chip, unsigned int gpio_line);
            ~FlowmeterComms();
        
        private:
            void read_pulse();

            std::atomic<bool> _running = false;
            std::thread reader_thread;

            const std::string gpio_chip_;
            unsigned int gpio_line_;

            int chip_fd_ = -1;
            int line_fd_ = -1;
    };
}
