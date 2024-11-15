/*
グローバル関数
can_init CAN通信の初期化を行う。can_sendとcan_recvをrtosのタスクで呼び出すための関数 pinを引数に取る
can_send CAN通信でデータを送信する
can_recv CAN通信でデータを受信する

グローバル変数
rx_data 受信したデータをid毎に格納する変数
tx_data 送信するデータをid毎に格納する変数

ここにアクチュエータの制御用クラスを書く。
class Motor{
Motor コンストラクタ can_id, speed_pid, location_pid, direction, reduction_ratio,
offset_angle, max_current, max_speedを引数に取る
コンストラクタ内でrx_data, tx_dataにidをキーとしたデータを追加する
set_speed_pid 速度制御のPIDパラメータを設定する
set_location_pid 位置制御のPIDパラメータを設定する
set_speed 速度を設定する
set_angle 位置を設定する 無限回転か、有限回転かを引数に取る
set_current 電流を設定する
get_current 電流を取得する
get_speed 速度を取得する
get_angle 位置を取得する
set_offset_angle オフセット角度を設定する

pid制御はrtosのタスクで行う
private:
rtos_func （必要なら）pid制御やデータの更新を行う関数
int ctrl_mode 制御モードを保存する変数
}
*/

#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP
// library includes
#include <Arduino.h>
#include <map>
#include "driver/twai.h"
#include "freertos/task.h"

// global variables
std::map<int, twai_message_t> rx_data;
std::map<int, twai_message_t> tx_data;

// global functions
void can_send_task(void* n){
    

/*
TX_PIN: CAN TX pin
RX_PIN: CAN RX pin
control_frequency: モータの制御周期 Hz

CAN通信の初期化を行う
*/
void can_init(uint8_t TX_PIN, uint8_t RX_PIN, int control_frequency){

}


#endif // MOTOR_CONTROL_HPP