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
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const int BAUDRATE = 115200;
const std::string SERIAL_PORT = "/dev/ttyUSB0";
// ********************************************************************************************************************
// クラスの定義
// ********************************************************************************************************************
class ReadSerialNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub;
    boost::asio::io_service io;
    boost::asio::serial_port port;
    boost::asio::streambuf buffer;
    std::string line; // シリアルポートから読み込んだデータを格納する変数
    // シリアルポートからの読み込みを開始する
    void start_read(){
        boost::asio::async_read_until(port, buffer, '\n',
            boost::bind(&ReadSerialNode::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred)
        );
    }
    // シリアルポートからの読み込みが完了したときに呼び出される
    void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred){
        if (!error){
            std::istream is(&buffer);
            std::getline(is, line);
            if(!line.empty()){
                // \nや\rを削除
                line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
                line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
                std::cout << line << std::endl;
                std_msgs::msg::Int32 num;
                // lineが数字の場合
                if(std::all_of(line.begin(), line.end(), ::isdigit)){
                    num.data = std::stoi(line);
                    pub->publish(num);
                }
            }
        }
        start_read();
    }

    public:
    ReadSerialNode() : Node("read_serial_node"), io(), port(io, SERIAL_PORT){
        std::cout << "call ReadSerialNode!" << std::endl;
        auto timer_callback = [this]() -> void{
            io.poll();
        };

        port.set_option(boost::asio::serial_port_base::baud_rate(BAUDRATE));
        start_read();
        pub = this->create_publisher<std_msgs::msg::Int32>("state_message", 10);
        timer = this->create_wall_timer(10ms, timer_callback);
    }

    ~ReadSerialNode(){
        port.close();
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ReadSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}