#ifndef _DJIMotorCtrlESP_HPP_
#define _DJIMotorCtrlESP_HPP_

#include <Arduino.h>
#include <map>
#include <functional>
#include "driver/twai.h"
#include "freertos/task.h"
#include "PID_CONTROL.hpp"
#include "adc_read.hpp"

class C600_DATA;
class MOTOR;
class M3508_P19;
class GM6020;

void can_init(uint8_t TX_PIN=8,uint8_t RX_PIN=18, int current_update_hz=200);
void location_contral_task(void* n);
void speed_contral_task(void* n);
void feedback_update_task(void* n);
void update_current_task(void* p);
void add_user_can_func(int addr,std::function<void(twai_message_t* can_message)> func);

class C600_DATA{
    friend void feedback_update_task(void* n);
    friend void update_current_task(void* n);
    friend void location_contral_task(void* n);
    friend void speed_contral_task(void* n);
    friend MOTOR;
    friend M3508_P19;
    friend GM6020;
public:
    C600_DATA(){}
    ~C600_DATA(){}
    float get_angle(){
        return 360.f*angle/8192.0;
    }
    int get_speed(){
        return speed;
    }
    int get_current(){
        return current;
    }
    int get_tempertrue(){
        return tempertrue;
    }
    int64_t get_location(){
        return location;
    }
    void reset_location(int l=0){
        location = l;
    }
    bool is_online(){
        return micros()-last_location_update_time<100000;
    }

protected:
    void update_location(){
        int16_t now_angle=angle;
        if (last_location_update_time==0){
            last_location_update_time=micros();
        }
        int now = micros();
        int delta=0;
        if((now_angle+8192-last_angle)%8192<4096){
            delta=now_angle-last_angle;
            if (delta<0){
                delta+=8192;
            }
        }else{
            delta=now_angle-last_angle;
            if (delta>0){
                delta-=8192;
            }
        }
        location += delta;
        last_location_update_time=now;
        last_angle=now_angle;
    }
    void update_data(twai_message_t can_message){
        angle = can_message.data[0]<<8 | can_message.data[1];
        speed = can_message.data[2]<<8 | can_message.data[3];
        current = can_message.data[4]<<8 | can_message.data[5];
        tempertrue = can_message.data[6];
        update_location();
    }
    uint16_t angle=0;
    int16_t speed=0;
    int16_t current=0;
    uint8_t tempertrue=0;
    int16_t set_current=0;
    int64_t location =0;
    bool enable=false;
    int64_t last_location_update_time=0;
    uint16_t last_angle=0;
    bool is_GM6020 = false;
};
C600_DATA motor_201,motor_202,motor_203,motor_204,motor_205,motor_206,motor_207,motor_208,motor_209,motor_20A,motor_20B;
C600_DATA* motors[]={&motor_201,&motor_202,&motor_203,&motor_204,&motor_205,&motor_206,&motor_207,&motor_208,&motor_209,&motor_20A,&motor_20B};

class MOTOR{
        friend void location_contral_task(void* n);
        friend void speed_contral_task(void* n);
        friend void update_current_task(void* n);
    public:
        MOTOR(){};
        MOTOR(int id){
            if(id<1 || id>8){
                return;
            }
            ID=id;
            data = motors[id-1];
            data->enable=true;
            location_pid_contraler.setPram(default_location_pid_parmater);
        }
        MOTOR(int id,pid_param location_pid,pid_param speed_pid){
            if(id<1 || id>8){
                return;
            }
            ID=id;
            data = motors[id-1];
            data->enable=true;
            location_pid_contraler.setPram(location_pid);
            speed_pid_contraler.setPram(speed_pid);
        }
        void setup(){
            data->enable=true;
            if(speed_func_handle==nullptr){
                xTaskCreate(speed_contral_task,"speed_contral_task",4096,this,2,&speed_func_handle);
            }
        };
        bool is_online(){
            return data->is_online();
        }
        void stop(bool need_unload=true){
            if(need_unload){
                unload();
            }
            location_taget=data->get_location();
            speed_location_taget=data->get_location();
            taget_speed=0;
            data->set_current=0;
        }
        void set_location_pid(float _location_Kp=0,float _location_Ki=0,float _location_Kd=0,float __dead_zone=0,float _max_speed=0){
                location_pid_contraler.Kp=_location_Kp;
                location_pid_contraler.Ki=_location_Ki;
                location_pid_contraler.Kd=_location_Kd;
                location_pid_contraler._dead_zone=__dead_zone;
                location_pid_contraler._max_value=_max_speed;
        }
        void set_location_pid(pid_param pid){
            set_location_pid(pid.Kp,pid.Ki,pid.Kd,pid._dead_zone,pid._max_value);
        }
        void set_speed_pid(float _speed_Kp=0,float _speed_Ki=0,float _speed_Kd=0,float __dead_zone=0,float _max_curunt=0){ 
                speed_pid_contraler.Kp=_speed_Kp;
                speed_pid_contraler.Ki=_speed_Ki;
                speed_pid_contraler.Kd=_speed_Kd;
                speed_pid_contraler._dead_zone=__dead_zone;
                max_curunt=_max_curunt;
                if(max_curunt>16384||max_curunt<=0){
                    max_curunt=16384;
                }
                speed_pid_contraler._max_value=_max_curunt;
                speed_pid_contraler.reset();
                location_pid_contraler.reset();
        }
        void set_speed_pid(pid_param pid){
            set_speed_pid(pid.Kp,pid.Ki,pid.Kd,pid._dead_zone,pid._max_value);
        }
        void set_location(int64_t _location){
            if(location_func_handle==nullptr){
                xTaskCreate(location_contral_task,"location_contral_task",4096,this,2,&location_func_handle);
            }
            location_taget=_location;
        }
        void reset_location(int64_t _location=0){
            data->reset_location(_location);
        }
        int64_t get_location(){
            return data->location;
        }
        int get_current_raw(){
            return data->get_current();
        }
        void set_max_curunt(float _max_curunt){
            if(_max_curunt>16384||max_curunt<=0)
                _max_curunt=16384;
            max_curunt=_max_curunt;
        }
        void unload(){
            if(speed_func_handle!=nullptr){
                vTaskDelete(speed_func_handle);
                speed_func_handle=nullptr;
            }
            if(location_func_handle!=nullptr){
                vTaskDelete(location_func_handle);
                location_func_handle=nullptr;
            }
            this->taget_speed=0;
            this->data->set_current=0;
            delay(30);
            this->data->enable=false;
        }
        void load(){
            taget_speed = 0;
            this->data->enable=true;
            if(speed_func_handle==nullptr){
                xTaskCreate(speed_contral_task,"speed_contral_task",4096,this,2,&speed_func_handle);
            }
        }
        bool get_is_load(){
            return speed_func_handle!=nullptr;
        }
        virtual float get_now_speed(){
            return data->speed;
        }
        virtual void set_speed(float speed,float acce=0){
            acce=acce>0?acce:0;
            this->data->enable=true;
            if(location_func_handle!=nullptr){
                vTaskDelete(location_func_handle);
                location_func_handle=nullptr;
            }
            if(speed_func_handle==nullptr){
                xTaskCreate(speed_contral_task,"speed_cspeed_func_handleontral_task",4096,this,2,&speed_func_handle);
            }
            taget_speed = speed;
            acceleration=acce;
        }
        float get_taget_speed(){
            return taget_speed;
        }
        void set_acceleration(float acce=0){
            acce=acce>0?acce:0;
            acceleration=acce;
        }
        float get_reduction_ratio(){
            return reduction_ratio;
        }
        void set_speed_location_K(float _K=1000){
            _K=_K>0?_K:-_K;
            speed_location_K=_K;
        }
        int get_control_frequency(){
            return control_frequency;
        }
        void set_control_frequency(int _control_frequency=200){
            if(_control_frequency<1){
                _control_frequency=50;
            }else if(_control_frequency>1000){
                _control_frequency=1000;
            }
            control_frequency=_control_frequency;
        }
        void add_location_to_current_func(std::function<int(int64_t)> func){
            location_to_current_func=func;
        }
        void add_location_to_current_func(int (*func_ptr)(int64_t location)){
                       location_to_current_func=func_ptr;
        }
    protected:
        std::function<int(int64_t)> location_to_current_func=null_location_to_current;
        uint8_t ID;
        static int null_location_to_current(int64_t location){
            return 0;
        }
        int64_t location_taget=0;
        int64_t speed_location_taget=0;
        pid_param default_location_pid_parmater={0.1,0.1,0,2000,3000};
        PID_CONTROL location_pid_contraler;
        pid_param default_speed_pid_parmater={5,1,0.01,1,10000};
        PID_CONTROL speed_pid_contraler;
        float max_curunt=10000;
        C600_DATA* data;
        float taget_speed = 0;
        TaskHandle_t location_func_handle = nullptr;
        TaskHandle_t speed_func_handle = nullptr;
        float reduction_ratio=1;
        float acceleration=0;
        int speed_location_K=1000;
        int control_frequency=200;
};

class M3508_P19:public MOTOR{
    public:
        M3508_P19(int id):MOTOR(id){
            reduction_ratio=19.0;
        };
        M3508_P19(int id,pid_param location_pid,pid_param speed_pid):MOTOR(id,location_pid,speed_pid){
            reduction_ratio=19.0;
        };
        void set_speed(float speed,float acce=0) override{
            acce=acce>0?acce:0;
            this->data->enable=true;
            if(location_func_handle!=nullptr){
                vTaskDelete(location_func_handle);
                location_func_handle=nullptr;
            }
            if(speed_func_handle==nullptr){
                xTaskCreate(speed_contral_task,"speed_cspeed_func_handleontral_task",4096,this,2,&speed_func_handle);
            }
            taget_speed = speed*19.0;
            acceleration=acce;
        }
        void set_angle(float angle,int8_t dir =0){
            angle-=angle_offset;
            float now_angle = get_adc_deg();
            float delta = angle - now_angle;
            if(abs(delta)>180&&dir==0){
                delta+=delta>0?-360:360;
            }
            delta_diff = delta - prior_delta;
            prior_delta = delta;
            delta_int += delta;
            float pid = delta*location_pid_contraler.Kp + delta_diff*location_pid_contraler.Kd + delta_int*location_pid_contraler.Ki;
            set_speed(max(min_speed, min(max_speed, pid)));
        }

        float get_curunt_ma(){
            return 2e4*data->current/16384;
        }

        float get_now_speed() override{
            return data->speed/19.0;
        }
    protected:
        float angle_offset = 0;
        float prior_delta = 0;
        float delta_diff = 0;
        float delta_int = 0;
        float max_speed = 300;
        float min_speed = -300;
};

class GM6020:public MOTOR{
    public:
        GM6020(int id){
            if(id<1 || id>7){
                return;
            }
            data=motors[id+3];
            data->enable=true;
            data->is_GM6020=true;
            pid_param speed_pid_parmater(10,0,0,1,16384);
            pid_param location_pid_parmater(0.2,0.1,0,5,350);
            location_pid_contraler.setPram(location_pid_parmater);
            speed_pid_contraler.setPram(speed_pid_parmater);
        };
        GM6020(int id,pid_param location_pid,pid_param speed_pid){
            if(id<1 || id>7){
                return;
            }
            data=motors[id+3];
            data->enable=true;
            data->is_GM6020=true;
            location_pid_contraler.setPram(location_pid);
            speed_pid_contraler.setPram(speed_pid);
        };
        float get_curunt_ma(){
            return 3e3*data->current/16384;
        }
        void set_angle(float angle,int8_t dir =0){
            angle-=angle_offset;
            reset_location(data->angle);
            float now_angle = get_angle(); // 0~360
            if (!(low_flag || high_flag) && prior_angle >= 0){
                if (prior_angle - now_angle > 180){
                    high_flag = true;
                    Serial.println("high true");
                }else if (prior_angle - now_angle < -180){
                    low_flag = true;
                    Serial.println("low true");
                }
            }
            prior_angle = now_angle;
            if (high_flag){
                if (now_angle > 180){
                    high_flag = false;
                    Serial.println("high false");
                }else{
                    now_angle += 360;
                }
            }
            if (low_flag){
                if (now_angle < 180){
                    low_flag = false;
                    Serial.println("low false");
                }else{
                    now_angle -= 360;
                }
            }
            angle=fmodf(angle,360.f); // -360~360
            angle=angle>=0?angle:360.f+angle; // 0~360
            if (angle <= 10) angle = 10;
            if (angle >= 350) angle = 350;
            float delta = angle - now_angle; // これで常に0~360の範囲で角度を取るようになるはず
            // Serial.printf("delta:%f\n\r", delta);

            // dir = dir > 0 ? 1 : (dir < 0 ? -1 : 0);
            // angle=fmodf(angle,360.f);
            // angle=angle>=0?angle:360.f+angle;
            // float delta = angle - now_angle;
            // while(dir*delta<0){
            //     delta+=dir*360;
            // }
            // if(abs(delta)>180&&dir==0){
            //     delta+=delta>0?-360:360;
            // }
            set_location(data->angle+delta*8192.f/360.f);
        }
        void set_angle_offset(float offset){
            if(offset<-180.f){
                while(offset<-180.f){
                    offset+=360.f;
                }
            }
            if(offset>180.f){
                while(offset>180.f){
                    offset-=360.f;
                }
            }
            angle_offset = offset;
        }
        float get_angle(){
            float angle_data=data->get_angle();
            angle_data+=angle_offset;
            angle_data=fmodf(angle_data,360.f);
            angle_data=angle_data>=0?angle_data:360.f+angle_data;
            return angle_data;
        }
    protected:
        float angle_offset = 0;
        float prior_angle = -1;
        bool low_flag = false;
        bool high_flag = false;
};

void location_contral_task(void* n){
    MOTOR* moto = (MOTOR*) n;
    moto->location_pid_contraler.reset();
    float speed=0;
    while (1){
        speed = moto->location_pid_contraler.control(moto->location_taget - moto->data->location);
        moto->taget_speed = speed;
        delay(1000/moto->control_frequency);
    }
};

void speed_contral_task(void* n){
    MOTOR* moto = (MOTOR*) n;
    int last_update_speed_time=micros();
    moto->speed_pid_contraler.reset();
    moto->speed_location_taget = moto->data->location;
    float taget_control_speed = moto->taget_speed;
    float last_taget_control_speed = moto->taget_speed;
    while (1){
        float delta_time=1e-6*(micros()-last_update_speed_time); 
        if(moto->acceleration==0){
            taget_control_speed = moto->taget_speed;
        }else{
            if(abs(taget_control_speed-last_taget_control_speed)>delta_time*moto->acceleration){
                taget_control_speed = last_taget_control_speed+(taget_control_speed-last_taget_control_speed)>0?delta_time*moto->acceleration:-delta_time*moto->acceleration;
            }else{
                taget_control_speed = taget_control_speed;
            }
        }
        last_taget_control_speed = taget_control_speed;
        moto->speed_location_taget+=moto->data->is_online()*8192*taget_control_speed*delta_time/60;
        last_update_speed_time=micros();
        double err = (taget_control_speed - moto->data->speed) + moto->speed_location_K * (moto->speed_location_taget-moto->data->location)/8192;
        int16_t cru = moto->location_to_current_func(moto->data->location) + moto->speed_pid_contraler.control(moto->data->is_online()*err);
        moto->data->set_current = cru;
        delay(1000/moto->control_frequency);
    }
}

std::map<int,std::function<void(twai_message_t* can_message)>> func_map;
void add_user_can_func(int addr,std::function<void(twai_message_t* can_message)> func){
    func_map[addr]=func;
};

void feedback_update_task(void* n){
    twai_message_t rx_message;
    while (1){
        ESP_ERROR_CHECK(twai_receive(&rx_message, portMAX_DELAY));
        if(rx_message.identifier>=0x201 && rx_message.identifier<=0x20B){
            motors[rx_message.identifier-0x201]->update_data(rx_message);
        }else if(func_map.find(rx_message.identifier)!=func_map.end()){
            func_map[rx_message.identifier](&rx_message);
            // Serial.printf("Receive CAN ID:%d\n",(rx_message.identifier & 0xff00) >> 8);
        }else{
            // Serial.printf("Unknown CAN ID:%d\n",rx_message.identifier);
        }
    }
}

void update_current_task(void* p){
    int frc=*(int*) p;
    while(1){
        if(motor_201.enable || motor_202.enable || motor_203.enable || motor_204.enable){
            twai_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x200;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = motor_201.set_current >> 8;
            tx_msg.data[1] = motor_201.set_current&0xff;
            tx_msg.data[2] = motor_202.set_current >> 8;
            tx_msg.data[3] = motor_202.set_current&0xff;
            tx_msg.data[4] = motor_203.set_current >> 8;
            tx_msg.data[5] = motor_203.set_current&0xff;
            tx_msg.data[6] = motor_204.set_current >> 8;
            tx_msg.data[7] = motor_204.set_current&0xff;
            esp_error_t ret = twai_transmit(&tx_msg,portMAX_DELAY);
            if (ret != ESP_OK){
                if (ret == ESP_ERROR_TIMEOUT){
                    Serial.println("CAN TX timeout");
                }else if (ret == ESP_FAIL){
                    Serial.println("CAN TX failed");
                }
            }
        }
        if(motor_205.enable || motor_206.enable || motor_207.enable || motor_208.enable){
            twai_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x1FF;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = motor_205.set_current >> 8;
            tx_msg.data[1] = motor_205.set_current&0xff;
            tx_msg.data[2] = motor_206.set_current >> 8;
            tx_msg.data[3] = motor_206.set_current&0xff;
            tx_msg.data[4] = motor_207.set_current >> 8;
            tx_msg.data[5] = motor_207.set_current&0xff;
            tx_msg.data[6] = motor_208.set_current >> 8;
            tx_msg.data[7] = motor_208.set_current&0xff;
            esp_error_t ret = twai_transmit(&tx_msg,portMAX_DELAY);
            if (ret != ESP_OK){
                if (ret == ESP_ERROR_TIMEOUT){
                    Serial.println("CAN TX timeout");
                }else if (ret == ESP_FAIL){
                    Serial.println("CAN TX failed");
                }
            }
        }
        if((motors[4]->enable&&motors[4]->is_GM6020)||(motors[5]->enable&&motors[5]->is_GM6020)||(motors[6]->enable&&motors[6]->is_GM6020)||(motors[7]->enable&&motors[7]->is_GM6020)){
            twai_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x1FE;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = motor_205.set_current >> 8;
            tx_msg.data[1] = motor_205.set_current&0xff;
            tx_msg.data[2] = motor_206.set_current >> 8;
            tx_msg.data[3] = motor_206.set_current&0xff;
            tx_msg.data[4] = motor_207.set_current >> 8;
            tx_msg.data[5] = motor_207.set_current&0xff;
            tx_msg.data[6] = motor_208.set_current >> 8;
            tx_msg.data[7] = motor_208.set_current&0xff;
            esp_error_t ret = twai_transmit(&tx_msg,portMAX_DELAY);
            if (ret != ESP_OK){
                if (ret == ESP_ERROR_TIMEOUT){
                    Serial.println("CAN TX timeout");
                }else if (ret == ESP_FAIL){
                    Serial.println("CAN TX failed");
                }
            }
        }
        if((motors[8]->enable&&motors[8]->is_GM6020)||(motors[9]->enable&&motors[9]->is_GM6020)||(motors[10]->enable&&motors[10]->is_GM6020)){
            twai_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x2FE;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = motor_209.set_current >> 8;
            tx_msg.data[1] = motor_209.set_current&0xff;
            tx_msg.data[2] = motor_20A.set_current >> 8;
            tx_msg.data[3] = motor_20A.set_current&0xff;
            tx_msg.data[4] = motor_20B.set_current >> 8;
            tx_msg.data[5] = motor_20B.set_current&0xff;
            tx_msg.data[6] = 0;
            tx_msg.data[7] = 0;
            esp_error_t ret = twai_transmit(&tx_msg,portMAX_DELAY);
            if (ret != ESP_OK){
                if (ret == ESP_ERROR_TIMEOUT){
                    Serial.println("CAN TX timeout");
                }else if (ret == ESP_FAIL){
                    Serial.println("CAN TX failed");
                }
            }
        }
        delay(1000/frc);
    }
}

void can_init(uint8_t TX_PIN, uint8_t RX_PIN,int current_update_hz){
    twai_status_info_t now_twai_status;
    auto status = twai_get_status_info(&now_twai_status);
    if(status != ESP_ERR_INVALID_STATE){
        Serial.println("can is installed do not need install again");
        return;
    }
    static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(TX_PIN), gpio_num_t(RX_PIN), TWAI_MODE_NO_ACK);
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
    xTaskCreate(feedback_update_task,"moto_fb",4096,nullptr,5,nullptr);
    xTaskCreate(update_current_task,"update_current_task",4096,&current_update_hz,5,nullptr);
    delay(100);
    Serial.println("finish can_init");
}
#endif
