/*
state_message
1. ROS実行成功
2. UDP成功
3. キャリブレーション完了
4. トレース開始
5. トレース終了
*/

// ライブラリのインクルードなど
// ********************************************************************************************************************
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include "std_msgs/msg/int8.hpp"
#include <thread>

using namespace std::chrono_literals;

const int BAUDRATE = 115200;
const std::string SERIAL_PORT = "/dev/ttyUSB0";

class ReadSerialNode : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub;
    boost::asio::io_service io;
    boost::asio::serial_port port;
    boost::asio::streambuf buffer;
    std::string line; // シリアルポートから読み込んだデータを格納する変数
    std_msgs::msg::Int8 num;
    std::thread io_thread;

    void start_read() {
        boost::asio::async_read_until(port, buffer, '\n',
            boost::bind(&ReadSerialNode::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred)
        );
    }

    void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred) {
        if (!error) {
            std::istream is(&buffer);
            std::getline(is, line);
            num.data = std::stoi(line);
            pub->publish(num);
            start_read();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error on receive: %s", error.message().c_str());
        }
    }

public:
    ReadSerialNode() : Node("read_serial_node"), port(io, SERIAL_PORT) {
        port.set_option(boost::asio::serial_port_base::baud_rate(BAUDRATE));
        pub = this->create_publisher<std_msgs::msg::Int8>("serial_data", 10);
        start_read();
        io_thread = std::thread([this]() { io.run(); });
    }

    ~ReadSerialNode() {
        io.stop();
        if (io_thread.joinable()) {
            io_thread.join();
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReadSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}