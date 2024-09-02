/*
CyberGearとの通信が確立したら，電流値を一定に保ちながら一定速度で回転させる．
CyberGearの角度を取得し続け，変化量がε以下になったら停止し，その角度を原点とする。
以上の動作をキャリブレーションとする．

その後，CyberGearは減速比に基づいて拡張位置制御を行う。

*/
#include <Arduino.h>
#include "cybergear_driver.hh"
#include "cybergear_can_interface_esp32.hh"