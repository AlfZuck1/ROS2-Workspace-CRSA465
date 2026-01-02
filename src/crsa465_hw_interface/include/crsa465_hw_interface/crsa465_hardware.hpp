#pragma once
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <deque>
#include <atomic>

#include "crsa465_interfaces/srv/command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <boost/asio.hpp>
#include <fcntl.h>


namespace crsa465_hw_interface
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

class CRSA465Hardware : public hardware_interface::SystemInterface
{
public:
    CRSA465Hardware() = default;
    ~CRSA465Hardware() override;

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    bool open_serial();
    void close_serial();
    void io_thread_fn();
    void process_rx_buffer();
    void publisher_thread_fn();
    bool send_command(const std::string & cmd);

    std::string port_;
    int baudrate_;
    double ticks_to_rad_{0.0};

    size_t n_joints_{0};
    std::vector<double> hw_positions_;
    std::vector<double> hw_commands_;
    std::vector<int32_t> last_counts_;
    std::mutex last_counts_mtx_;

    boost::asio::io_service io_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::vector<uint8_t> rx_buffer_;
    std::mutex rx_mtx_;
    std::deque<std::vector<uint8_t>> tx_queue_;
    std::mutex tx_mtx_;
    std::thread io_thread_;
    std::atomic<bool> io_running_{false};

    static constexpr uint8_t HDR_RX0 = 0xFD;
    static constexpr uint8_t HDR_RX1 = 0xFC;
    static constexpr uint8_t HDR_TX0 = 0xAA;
    static constexpr uint8_t HDR_TX1 = 0xFE;
    static constexpr uint8_t CMD_HEADER = 0xAA;
    static constexpr uint8_t CMD_STOP = 0x01;
    static constexpr uint8_t CMD_RUN = 0x02;
    static constexpr uint8_t CMD_HOME = 0x03;
    static constexpr uint8_t CMD_CALIBRATE = 0x04;
    static constexpr size_t STATE_PACKET_SIZE = 2 + 6 * 4;

    // Publisher
    std::shared_ptr<rclcpp::Node> pub_node_;
    rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_pub_;
    rclcpp::Service<crsa465_interfaces::srv::Command>::SharedPtr command_srv_;
    std::thread pub_thread_;
    std::atomic<bool> pub_running_{false};
};
} // namespace
