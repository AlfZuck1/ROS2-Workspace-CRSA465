#include "crsa465_hw_interface/crsa465_hardware.hpp"
#include <chrono>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace crsa465_hw_interface
{
    using namespace std::chrono_literals;

    CRSA465Hardware::~CRSA465Hardware()
    {
        io_running_ = false;
        if (io_thread_.joinable()) io_thread_.join();

        pub_running_ = false;
        if (pub_thread_.joinable()) pub_thread_.join();

        close_serial();
    }

    CallbackReturn CRSA465Hardware::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        n_joints_ = info.joints.size();
        hw_positions_.assign(n_joints_, 0.0);
        hw_commands_.assign(n_joints_, 0.0);
        last_counts_.assign(n_joints_, 0);

        port_ = info.hardware_parameters.count("port") ? info.hardware_parameters.at("port") : "/dev/ttyACM0";
        baudrate_ = info.hardware_parameters.count("baudrate") ? std::stoi(info.hardware_parameters.at("baudrate")) : 115200;
        ticks_to_rad_ = info.hardware_parameters.count("ticks_to_rad") ? std::stod(info.hardware_parameters.at("ticks_to_rad")) : 1.0;

        RCLCPP_INFO(rclcpp::get_logger("CRS_HW"), "on_init: port=%s baud=%d joints=%zu ticks_to_rad=%f", port_.c_str(), baudrate_, n_joints_, ticks_to_rad_);

        if (!open_serial()) {
            RCLCPP_ERROR(rclcpp::get_logger("CRS_HW"), "Failed to open serial port %s", port_.c_str());
            return CallbackReturn::ERROR;
        }

        io_running_ = true;
        io_thread_ = std::thread(&CRSA465Hardware::io_thread_fn, this);

        // Crear nodo interno para publicar controller_state
        pub_node_ = std::make_shared<rclcpp::Node>("crsa465_hw_publisher_node");
        state_pub_ = pub_node_->create_publisher<control_msgs::msg::JointTrajectoryControllerState>("arm_controller/controller_state", 10);

        command_srv_ = pub_node_->create_service<crsa465_interfaces::srv::Command>(
        "command_controller",
        [this](
            const std::shared_ptr<crsa465_interfaces::srv::Command::Request> request,
            std::shared_ptr<crsa465_interfaces::srv::Command::Response> response) 
        {
            RCLCPP_INFO(pub_node_->get_logger(), "Received command: %s", request->command.c_str());
            bool ok = this->send_command(request->command);
            response->success = ok;
            response->message = ok ? "Command executed successfully" : "Failed to execute command";
        });

        pub_running_ = true;
        pub_thread_ = std::thread(&CRSA465Hardware::publisher_thread_fn, this);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> CRSA465Hardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> ret;
        for (size_t i = 0; i < n_joints_; ++i)
            ret.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "position", &hw_positions_[i]));
        return ret;
    }

    std::vector<hardware_interface::CommandInterface> CRSA465Hardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> ret;
        for (size_t i = 0; i < n_joints_; ++i)
            ret.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "position", &hw_commands_[i]));
        return ret;
    }

    return_type CRSA465Hardware::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // std::lock_guard<std::mutex> lk(last_counts_mtx_);
        // for (size_t i = 0; i < n_joints_; ++i)
        //     hw_positions_[i] = last_counts_[i] * ticks_to_rad_;
        return return_type::OK;
    }

    return_type CRSA465Hardware::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        static int contador = 0;
        contador++;
        std::vector<uint8_t> pkt;
        pkt.reserve(2 + n_joints_ * sizeof(float) * 2);
        pkt.push_back(HDR_TX0);
        pkt.push_back(HDR_TX1);

        for (size_t i = 0; i < n_joints_; ++i) {
            // TODO: convertir el 10000 de manera correcta
            float pos = static_cast<float>(hw_commands_[i]*10000);
            uint8_t bpos[4];
            std::memcpy(bpos, &pos, sizeof(float));
            pkt.insert(pkt.end(), bpos, bpos + 4);
        }
        
        for (size_t i = 0; i < n_joints_; ++i) {
            float vel = static_cast<float>(100.0);
            uint8_t bvel[4];
            std::memcpy(bvel, &vel, sizeof(float));
            pkt.insert(pkt.end(), bvel, bvel + 4);
        }

        if(contador % 200 == 0){
            RCLCPP_INFO(rclcpp::get_logger("CRS_HW"), "WRITE called: %f %f %f %f %f %f",
            hw_commands_[0], hw_commands_[1], hw_commands_[2],
            hw_commands_[3], hw_commands_[4], hw_commands_[5]);
        }

        std::lock_guard<std::mutex> lk(tx_mtx_);
        std::stringstream ss;
        ss << "Enqueuing packet: ";
        for (auto b : pkt) ss << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        // RCLCPP_INFO(rclcpp::get_logger("CRS_HW"), "%s", ss.str().c_str());
        tx_queue_.push_back(std::move(pkt));

        return return_type::OK;
    }

    void CRSA465Hardware::publisher_thread_fn()
    {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(pub_node_);

        rclcpp::Rate rate(50); // 50 Hz
        while (pub_running_ && rclcpp::ok()) {
            control_msgs::msg::JointTrajectoryControllerState msg;
            msg.joint_names = {"junta_0_1","junta_1_2","junta_2_3","junta_3_4","junta_4_5","junta_5_6"};

            {
                std::lock_guard<std::mutex> lk(last_counts_mtx_);
                // Posiciones reales medidas
                msg.actual.positions = hw_positions_;
                msg.actual.velocities.resize(n_joints_, 0.0);
                msg.actual.accelerations.resize(n_joints_, 0.0);
                msg.actual.effort.resize(n_joints_, 0.0);

                // Posiciones deseadas enviadas al hardware
                msg.desired.positions = hw_commands_;
                msg.desired.velocities.resize(n_joints_, 0.0);
                msg.desired.accelerations.resize(n_joints_, 0.0);

                // Error
                msg.error.positions.resize(n_joints_);
                for (size_t i = 0; i < n_joints_; ++i)
                    msg.error.positions[i] = msg.desired.positions[i] - msg.actual.positions[i];
                msg.error.velocities.resize(n_joints_, 0.0);
                msg.error.accelerations.resize(n_joints_, 0.0);
            }

            msg.header.stamp = pub_node_->now();
            state_pub_->publish(msg);

            exec.spin_some();
            rate.sleep();
        }
    }


    /* ---------------- IO thread ---------------- */

    void CRSA465Hardware::io_thread_fn()
    {
        RCLCPP_INFO(rclcpp::get_logger("CRS_HW"), "Serial IO thread started");

        while (io_running_) {
            try {
                static int contador = 0;
                contador++;
                // ---- Enviar paquetes pendientes ----
                {
                    std::lock_guard<std::mutex> lk(tx_mtx_);
                    while (!tx_queue_.empty()) {
                        auto p = std::move(tx_queue_.front());
                        tx_queue_.pop_front();

                        try {
                            // boost::asio::write asegura envío completo
                            size_t bytes_written = boost::asio::write(*serial_port_, boost::asio::buffer(p));
                            if(contador % 300 == 0){
                                RCLCPP_INFO(rclcpp::get_logger("CRS_HW"), "WRITE in IO thread: %zu bytes", bytes_written);
                            }
                        } catch (const std::exception &e) {
                            RCLCPP_ERROR(rclcpp::get_logger("CRS_HW"), "IO write exception: %s", e.what());
                        }
                    }
                }

                // ---- Leer datos de serial ----
                uint8_t buf[256];
                boost::system::error_code ec;
                size_t n = serial_port_->read_some(boost::asio::buffer(buf), ec);

                if (!ec && n > 0) {
                    // Guardar bytes recibidos
                    {
                        std::lock_guard<std::mutex> lk(rx_mtx_);
                        rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);
                    }

                    // Log opcional
                    if (contador % 300 == 0) {
                        std::stringstream ss;
                        ss << "Received " << n << " bytes: ";
                        for (size_t i = 0; i < n; ++i)
                            ss << std::hex << std::setw(2) << std::setfill('0')
                            << static_cast<int>(buf[i]) << " ";
                        // RCLCPP_INFO(rclcpp::get_logger("CRS_HW"), "%s", ss.str().c_str());
                    }
                    process_rx_buffer();
                }

                std::this_thread::sleep_for(2ms);

            } catch (const std::exception & e) {
                RCLCPP_ERROR(rclcpp::get_logger("CRS_HW"), "IO thread exception: %s", e.what());
                std::this_thread::sleep_for(100ms);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("CRS_HW"), "Serial IO thread stopped");
    }

    void CRSA465Hardware::process_rx_buffer()
    {
        std::lock_guard<std::mutex> lk(rx_mtx_);

        static const uint8_t HEADER[2] = {HDR_RX0, HDR_RX1};
        const size_t PACKET_SIZE = 26;

        while (true)
        {
            // Buscar el header FD FC
            auto it = std::search(
                rx_buffer_.begin(),
                rx_buffer_.end(),
                std::begin(HEADER),
                std::end(HEADER)
            );

            if (it == rx_buffer_.end()) {
                // No hay header -> esperar más datos
                return;
            }

            size_t pos = std::distance(rx_buffer_.begin(), it);

            // Verificar que hay suficientes bytes para un paquete entero
            if (rx_buffer_.size() < pos + PACKET_SIZE)
                return;  // Llegará más data

            // Extraer paquete
            std::vector<uint8_t> pkt(it, it + PACKET_SIZE);

            // Eliminar bytes previos + paquete completo
            rx_buffer_.erase(rx_buffer_.begin(), it + PACKET_SIZE);

            // Parsear floats
            float vals[6];
            for (size_t j = 0; j < 6; ++j) {
                std::memcpy(&vals[j], pkt.data() + 2 + j * 4, sizeof(float));
            }

            // Guardar en last_counts_
            {
                std::lock_guard<std::mutex> lk2(last_counts_mtx_);
                for (size_t j = 0; j < n_joints_; ++j) 
                {
                    last_counts_[j] = vals[j];
                    hw_positions_[j] = vals[j] * ticks_to_rad_;
                }
            }

            // Log
            RCLCPP_INFO(
                rclcpp::get_logger("CRS_HW"),
                "RX encoders: %f %f %f %f %f %f",
                vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]
            );
        }
    }

    bool CRSA465Hardware::open_serial()
    {
    try {
        serial_port_.reset(new boost::asio::serial_port(io_));
        serial_port_->open(port_);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        int fd = serial_port_->native_handle();
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        RCLCPP_INFO(rclcpp::get_logger("CRS_HW"), "Opened serial %s @ %d (non-blocking via fcntl)", port_.c_str(), baudrate_);
        return true;
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("CRS_HW"), "open_serial exception: %s", e.what());
        return false;
    }
    }

    void CRSA465Hardware::close_serial()
    {
    try {
        if (serial_port_ && serial_port_->is_open()) serial_port_->close();
        serial_port_.reset();
    } catch (...) {}
    }

    bool CRSA465Hardware::send_command(const std::string & cmd)
    {
        uint8_t command_byte = 0x00;

        if (cmd == "stop") command_byte = CMD_STOP;
        else if (cmd == "home") command_byte = CMD_HOME;
        else if (cmd == "reset") command_byte = CMD_RESET;
        else 
        {
            RCLCPP_WARN(rclcpp::get_logger("CRS_HW"), "Unknown command: %s", cmd.c_str());
            return false;
        }

        std::vector<uint8_t> pkt;
        pkt.reserve(2);
        pkt.push_back(CMD_HEADER);
        pkt.push_back(command_byte);

        // Encolar de forma thread-safe para que el io_thread lo envíe
        {
            std::lock_guard<std::mutex> lk(tx_mtx_);
            tx_queue_.push_back(std::move(pkt));
        }

        RCLCPP_INFO(rclcpp::get_logger("CRS_HW"),"Enqueued command '%s' (0x%02X)", cmd.c_str(), command_byte);
        return true;
    }

} // namespace

// plugin export
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(crsa465_hw_interface::CRSA465Hardware, hardware_interface::SystemInterface)
