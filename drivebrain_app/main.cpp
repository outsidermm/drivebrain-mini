#include <FoxgloveServer.hpp>
#include <MCAPLogger.hpp>
#include <drivebrain_msgs.pb.h>
#include <FlowmeterComms.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>
#include <fmt/chrono.h>
#include <spdlog/spdlog.h>
#include <memory>

std::atomic<bool> running{true};

void sig_handler(int signal) {
    if(signal == SIGINT || signal == SIGTERM) {
      spdlog::warn("Interrupted, stopping mini-drivebrain");
      running = false;
    }
}

int main() {
  constexpr char FLOWMETER_GPIO_CHIP[] = "/dev/gpiochip0";
  constexpr unsigned int FLOWMETER_GPIO_LINE = 17;
  constexpr double PULSE_PER_LITER = 450.0;

  std::signal(SIGINT, sig_handler);
  std::signal(SIGTERM, sig_handler);

  core::MCAPLogger::create(mcap::McapWriterOptions(""));
  core::FoxgloveServer::create();

  core::MCAPLogger::instance().open_new_mcap();
  core::MCAPLogger::instance().init_logging();

  spdlog::info("Constructed logging singletons");

  auto loop_time = 0.004;
  std::chrono::microseconds loop_time_ms((int) (loop_time * 1000000.0f));
  auto next_tick = std::chrono::steady_clock::now();
  {
    comms::FlowmeterComms flowmeter(FLOWMETER_GPIO_CHIP, FLOWMETER_GPIO_LINE);
    std::uint64_t previous_count = flowmeter.get_pulse_count();
    auto previous_time = std::chrono::steady_clock::now();
    while(running) {

      next_tick += loop_time_ms;

      auto now = std::chrono::steady_clock::now();
      if(now > next_tick) {
        spdlog::warn("Loop overrun by {}", now-next_tick);
        next_tick = now;
      }

      if (now - previous_time >= std::chrono::seconds(1)) {
        const std::uint64_t current_count = flowmeter.get_pulse_count();
        const std::uint64_t delta_count = current_count - previous_count;
        const double elapsed_time = std::chrono::duration<double>(now - previous_time).count();
        const double liters_per_minute = delta_count / (PULSE_PER_LITER * elapsed_time) * 60.0;

        spdlog::info("Pulse count: {}", flowmeter.get_pulse_count());
        spdlog::info("Liters per minute: {}", liters_per_minute);

        auto message = std::make_shared<drivebrain_msgs::FlowmeterData>();
        message->set_flow_liters_per_minute(liters_per_minute);
        message->set_pulse_count(current_count);

        core::MCAPLogger::instance().log_msg(message);
        core::FoxgloveServer::instance().send_live_telem_msg(message);
        
        previous_count = current_count;
        previous_time = now;
      }



      std::this_thread::sleep_until(next_tick);
    }
  }

  core::MCAPLogger::instance().destroy();
  core::FoxgloveServer::instance().destroy();
  spdlog::info("Logging singletons safely destroyed");

  return 0;
}
