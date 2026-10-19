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

            try {
                processing_thread = std::thread(&FlowmeterComms::process_pulse, this);
            } catch (...) {
                _running = false;
                if (reader_thread.joinable()) reader_thread.join();
                close(line_fd_);
                line_fd_ = -1;
                close(chip_fd_);
                chip_fd_ = -1;
                throw;
            }
        }
    

    void FlowmeterComms::read_pulse() {
        constexpr int POLL_TIMEOUT_MS = 100;
        gpio_v2_line_event event{};
        pollfd descriptor{};
        descriptor.fd = line_fd_;
        descriptor.events = POLLIN;

        while(_running) {
            int poll_result = poll(&descriptor, 1, POLL_TIMEOUT_MS);

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
                {
                    std::lock_guard<std::mutex> lock(pulse_queue_.mtx);
                    if (!_running) break;
                    pulse_queue_.deque.push_back(event);
                }
                pulse_queue_.cv.notify_one();
            }
        }
    }

    void FlowmeterComms::process_pulse() {
        while (true) {
            std::deque<gpio_v2_line_event> pending;
            {
                std::unique_lock<std::mutex> lock(pulse_queue_.mtx);
                pulse_queue_.cv.wait(lock, [this]() {
                    return !_running || !pulse_queue_.deque.empty();
                });

                if (!_running && pulse_queue_.deque.empty()) break;
                pending.swap(pulse_queue_.deque);
            }

            pulse_count_+= pending.size();

        }
    }

    std::uint64_t FlowmeterComms::get_pulse_count() const {
        return pulse_count_.load();
    }

    FlowmeterComms::~FlowmeterComms() {
        {
            std::lock_guard<std::mutex> lock(pulse_queue_.mtx);
            _running = false;

        }
        if (reader_thread.joinable()) reader_thread.join();
        pulse_queue_.cv.notify_all();
        if(processing_thread.joinable()) processing_thread.join();
        if (line_fd_ >= 0) close(line_fd_);
        if (chip_fd_ >= 0) close(chip_fd_);
    }
};