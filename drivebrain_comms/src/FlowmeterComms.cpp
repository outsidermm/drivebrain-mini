#include <FlowmeterComms.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <cstring>
#include <sys/ioctl.h>
#include <poll.h>
#include <cerrno>

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
            
            try {
                reader_thread = std::thread(&FlowmeterComms::read_pulse, this);
            } catch (...) {
                _running = false;
                close(line_fd_);
                line_fd_ = -1;
                close(chip_fd_);
                chip_fd_ = -1;
                throw;
            }
        }
    

    void FlowmeterComms::read_pulse() {
        gpio_v2_line_event event{};
        pollfd descriptor{};
        descriptor.fd = line_fd_;
        descriptor.events = POLLIN;

        while(_running) {
            int poll_result = poll(&descriptor, 1, 100);

            if (poll_result == 0) continue;
            if (poll_result == -1) {
                if (errno == EINTR) continue;
                break;
            }

            if(descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                break;
            }

            if (descriptor.revents & POLLIN) {
                ssize_t read_result = read(line_fd_, &event, sizeof(event));
                if (read_result == -1 && errno == EINTR) continue;
                if (read_result != sizeof(event)) break;
                // Process the received event
            }
        }
    }

    FlowmeterComms::~FlowmeterComms() {
        _running = false;
        if (reader_thread.joinable()) reader_thread.join();
        if (line_fd_ >= 0) close(line_fd_);
        if (chip_fd_ >= 0) close(chip_fd_);
    }
};