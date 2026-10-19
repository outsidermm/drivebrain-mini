#include <FlowmeterComms.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <cstring>
#include <sys/ioctl.h>

namespace comms {
    FlowmeterComms::FlowmeterComms(
        const std::string& gpio_chip, unsigned int gpio_line) : gpio_chip_(gpio_chip), gpio_line_(gpio_line) {
            chip_fd_ = open(gpio_chip_.c_str(), O_RDONLY);
            if (chip_fd_ < 0) {
                throw std::runtime_error("Failed to open GPIO chip: " + gpio_chip_);
            }
            gpio_v2_line_request request{};
            request.offsets[0]=gpio_line_;
            request.num_lines=1;
            std::strcpy(request.consumer, "FlowmeterComms");
            request.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING;
            int result = ioctl(chip_fd_, GPIO_V2_GET_LINE_IOCTL, &request);
            if (result == -1) {
                close(chip_fd_);
                chip_fd_ = -1;
                throw std::runtime_error("Failed to request GPIO line: " + std::to_string(gpio_line_));
            }
            line_fd_ = request.fd;
            _running = true;
        }
    

    FlowmeterComms::~FlowmeterComms() {
        if (line_fd_ >= 0) {
            close(line_fd_);
        }

        if (chip_fd_ >= 0) {
            close(chip_fd_);
        }

        _running = false;
    }
};