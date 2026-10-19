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
    comms::FlowmeterComms flowmeter("/dev/gpiochip0", 17);

    while(running) {

      next_tick += loop_time_ms;

      auto now = std::chrono::steady_clock::now();
      if(now > next_tick) {
        spdlog::warn("Loop overrun by {}", now-next_tick);
        next_tick = now;
      }

      spdlog::info("Pulse count: {}", flowmeter.get_pulse_count());

      auto message = std::make_shared<drivebrain_msgs::FlowmeterData>();
      message->set_pulse_count(flowmeter.get_pulse_count());

      core::MCAPLogger::instance().log_msg(message);
      core::FoxgloveServer::instance().send_live_telem_msg(message);

      std::this_thread::sleep_until(next_tick);
    }
  }

  core::MCAPLogger::instance().destroy();
  core::FoxgloveServer::instance().destroy();
  spdlog::info("Logging singletons safely destroyed");

  return 0;
}
