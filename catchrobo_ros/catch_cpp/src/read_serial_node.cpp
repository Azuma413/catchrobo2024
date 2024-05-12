// できたらロボマス側のノードとコンポーネント化したい。
// ライブラリのインクルードなど
// ********************************************************************************************************************
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

using namespace std::chrono_literals;
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************

// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class ReadSerialNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
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
                std::cout << line << std::endl;
            }
        }
        start_read();
    }

    public:
    ReadSerialNode() : Node("read_serial_node"), io(), port(io, "/dev/ttyUSB0"){
        std::cout << "call ReadSerialNode!" << std::endl;
        auto timer_callback = [this]() -> void{
            io.poll();
        };

        port.set_option(boost::asio::serial_port_base::baud_rate(115200));
        start_read();
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