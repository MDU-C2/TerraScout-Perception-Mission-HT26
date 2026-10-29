"""Run drive freshness/encoding/limit tests without ESP-IDF or motors."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="terrascout-drive-") as directory:
    tmp = Path(directory)
    for name in ("drive_control.hpp", "motor_interface.hpp", "config.h", "nuc_protocol.hpp", "bms_interface.hpp"):
        shutil.copy(root / "include" / name, tmp / name)
    for name in ("drive_control.cpp", "motor_interface.cpp"):
        shutil.copy(root / "src" / name, tmp / name)
    (tmp / "driver").mkdir()
    (tmp / "driver/gpio.h").write_text("""#pragma once
using gpio_num_t=int;
enum {GPIO_NUM_4=4,GPIO_NUM_5=5,GPIO_NUM_6=6,GPIO_NUM_17=17,GPIO_NUM_18=18};
""")
    (tmp / "driver/uart.h").write_text("""#pragma once
using uart_port_t=int; enum {UART_NUM_0=0,UART_NUM_1=1};
""")
    (tmp / "can.hpp").write_text("""#pragma once
#include <cstdint>
#include <vector>
using esp_err_t=int;
constexpr int ESP_OK=0, ESP_ERR_TIMEOUT=1;
inline const char* esp_err_to_name(int) {return "stub";}
class Can {public:
struct Frame {uint32_t id=0;bool extended=false;bool rtr=false;
uint8_t dlc=0;uint8_t data[8]{};};
std::vector<Frame> sent;
int send(const Frame& f,int){sent.push_back(f);return ESP_OK;}
int begin(int,int,uint32_t,unsigned=16){return ESP_OK;}
int receive(Frame&,uint32_t){return ESP_ERR_TIMEOUT;}
bool isBusOff() const {return false;}
int recover(){return ESP_OK;}
void logDiagnostics(){}
};
""")
    (tmp / "sbus.hpp").write_text("""#pragma once
#include "config.h"
struct SbusFrame {uint16_t channels[16]{};bool valid=false;
bool frame_lost=false;bool failsafe=false;};
class Sbus {public:Sbus(int,int){} bool read(SbusFrame&,int){return false;}
int64_t last_frame_time_us()const{return 0;}
unsigned good_frames()const{return 0;} unsigned bad_windows()const{return 0;}};
""")
    (tmp / "esp_log.h").write_text("#pragma once\n#define ESP_LOGW(...) do {} while(0)\n#define ESP_LOGI(...) do {} while(0)\n#define ESP_LOGE(...) do {} while(0)\n")
    (tmp / "uart.hpp").write_text("#pragma once\n#include <cstdint>\nclass Uart {public: Uart(int,int,int,int){} int receive_exact(uint8_t*,unsigned,int){return 0;}};\n")
    (tmp / "freertos").mkdir()
    (tmp / "freertos/FreeRTOS.h").write_text("""#pragma once
using QueueHandle_t=void*; using TickType_t=unsigned;
constexpr int pdTRUE=1,pdPASS=1;
inline unsigned pdMS_TO_TICKS(unsigned n){return n;}
inline void* xQueueCreate(unsigned,unsigned){return nullptr;}
inline int xQueueReceive(void*,void*,unsigned){return 0;}
inline int xQueueOverwrite(void*,const void*){return 1;}
inline unsigned xTaskGetTickCount(){return 0;}
inline void vTaskDelete(void*){}
inline void vTaskDelayUntil(unsigned*,unsigned){}
inline int xTaskCreate(void(*)(void*),const char*,unsigned,void*,unsigned,void*){return 1;}
""")
    for name in ("queue.h", "task.h"):
        (tmp / "freertos" / name).write_text('#include "FreeRTOS.h"\n')
    (tmp / "esp_timer.h").write_text("""#pragma once
#include <cstdint>
extern int64_t test_now;
inline int64_t esp_timer_get_time(){return test_now;}
""")
    (tmp / "test.cpp").write_text(r'''
#include "drive_control.hpp"
#include <cassert>
#include <cmath>
#include <limits>
int64_t test_now=1000;
static int32_t value(const Can::Frame &f) {
    const uint32_t raw=(uint32_t(f.data[0])<<24)|(uint32_t(f.data[1])<<16)|
                       (uint32_t(f.data[2])<<8)|f.data[3];
    return static_cast<int32_t>(raw);
}
int main() {
    ControllerInput rc{};
    rc.frame.valid=true; rc.received_at_us=1000;
    rc.frame.channels[config::SBUS_MODE_CHANNEL]=992;
    assert(mode_from_controller(rc)==DriveMode::MANUAL);
    test_now=1000+config::CONTROLLER_INPUT_TIMEOUT_US;
    assert(mode_from_controller(rc)==DriveMode::DISABLED); // no new frame

    NucInput nuc{};
    assert(!nuc_input_fresh(nuc,1000));
    nuc.received_at_us=1000;
    assert(nuc_input_fresh(nuc,1000));
    assert(nuc_input_fresh(nuc,1000+config::NUC_INPUT_TIMEOUT_US-1));
    assert(!nuc_input_fresh(nuc,1000+config::NUC_INPUT_TIMEOUT_US));
    assert(!nuc_input_fresh(nuc,999));
    nuc.received_at_us=-1;
    assert(!nuc_input_fresh(nuc,1000));

    Can can; SendFailure failure;
    assert(send_drive_erpm(can,std::numeric_limits<int32_t>::max(),
                           std::numeric_limits<int32_t>::min(),failure));
    assert(can.sent.size()==4);
    for(unsigned i=0;i<4;++i)
        assert(value(can.sent[i])==(i<2?config::MAX_MOTOR_ERPM:-config::MAX_MOTOR_ERPM));
    can.sent.clear();
    // Exercise the same age gate and zero-output path used by control_task.
    nuc.left_erpm=-12; nuc.right_erpm=34; nuc.received_at_us=1000;
    const int64_t times[]={1001,1050,1000+config::NUC_INPUT_TIMEOUT_US};
    for(const auto now:times) {
        const bool fresh=nuc_input_fresh(nuc,now);
        assert(send_drive_erpm(can,fresh?nuc.left_erpm:0,fresh?nuc.right_erpm:0,failure));
        assert(value(can.sent[can.sent.size()-4])==(fresh?-12:0));
        assert(value(can.sent.back())==(fresh?34:0));
    }
    can.sent.clear();
    assert(send_drive(can,0.1,0.1,failure));
    assert(value(can.sent[0])<0 && value(can.sent[2])>0);

    DriveEnableGate gate;
    rc.frame.frame_lost=false; rc.frame.failsafe=false;
    test_now=1000; rc.received_at_us=1000;
    rc.frame.channels[config::SBUS_THROTTLE_CHANNEL]=config::SBUS_CHANNEL_MID;
    rc.frame.channels[config::SBUS_STEERING_CHANNEL]=config::SBUS_CHANNEL_MID;
    rc.frame.channels[config::SBUS_MODE_CHANNEL]=992;
    assert(gate.update(rc)==DriveMode::DISABLED); // boot in manual cannot arm
    rc.frame.channels[config::SBUS_MODE_CHANNEL]=172;
    assert(gate.update(rc)==DriveMode::DISABLED); // disable + neutral arms
    rc.frame.channels[config::SBUS_MODE_CHANNEL]=992;
    assert(gate.update(rc)==DriveMode::MANUAL);
    rc.frame.channels[config::SBUS_MODE_CHANNEL]=1811;
    assert(gate.update(rc)==DriveMode::DISABLED); // no direct manual -> auto
    assert(gate.update(rc)==DriveMode::DISABLED);
    rc.frame.channels[config::SBUS_MODE_CHANNEL]=172;
    gate.update(rc);
    rc.frame.channels[config::SBUS_MODE_CHANNEL]=1811;
    assert(gate.update(rc)==DriveMode::AUTONOMOUS);
    test_now=201000;
    assert(gate.update(rc)==DriveMode::DISABLED);
    test_now=202000;rc.received_at_us=202000;
    assert(gate.update(rc)==DriveMode::DISABLED); // reconnect does not rearm

    const auto mid=config::SBUS_CHANNEL_MID;
    const auto min=config::SBUS_CHANNEL_MIN;
    const auto max=config::SBUS_CHANNEL_MAX;
    ManualRamp ramp;
    const double full = config::MAX_MOTOR_ERPM * config::TWO_PI / 60.0 /
                        (config::POLE_PAIRS * config::GEAR_RATIO);
    auto ramped = ramp.update({full,-full},0.05);
    assert(std::abs(ramped.left_rad_s-full*0.05/config::MANUAL_RAMP_SECONDS)<1e-9);
    assert(ramped.right_rad_s == -ramped.left_rad_s);
    for(int i=0;i<40;++i) ramped=ramp.update({full,-full},0.05);
    assert(std::abs(ramped.left_rad_s-full)<1e-9);
    ramped=ramp.update({-full,full},0.05);
    assert(ramped.left_rad_s>0); // reversal cannot jump sign
    ramped=ramp.update({},0.05);
    assert(ramped.left_rad_s==0 && ramped.right_rad_s==0);
    ramped=ramp.update({full,full},5.0);
    assert(ramped.left_rad_s<=full*0.05/config::MANUAL_RAMP_SECONDS+1e-9);
    ramp.reset();
    WheelVelocities wheels;
    assert(manual_wheel_velocities(mid,mid,wheels));
    assert(wheels.left_rad_s==0 && wheels.right_rad_s==0);
    assert(manual_wheel_velocities(mid+config::SBUS_DEADBAND,
                                   mid-config::SBUS_DEADBAND,wheels));
    assert(wheels.left_rad_s==0 && wheels.right_rad_s==0);
    assert(manual_wheel_velocities(max,mid,wheels));
    assert(wheels.left_rad_s>0 && wheels.left_rad_s==wheels.right_rad_s);
    const double limit=wheels.left_rad_s;
    assert(manual_wheel_velocities(min,mid,wheels));
    assert(wheels.left_rad_s<0 && wheels.left_rad_s==wheels.right_rad_s);
    assert(manual_wheel_velocities(mid,max,wheels));
    assert(wheels.left_rad_s==-wheels.right_rad_s);
    assert(wheels.left_rad_s*config::SBUS_STEERING_SIGN>0);
    assert(manual_wheel_velocities(mid,min,wheels));
    assert(wheels.left_rad_s*config::SBUS_STEERING_SIGN<0);
    assert(manual_wheel_velocities(max,max,wheels));
    assert(std::abs(wheels.left_rad_s)<=limit && std::abs(wheels.right_rad_s)<=limit);
    assert(!manual_wheel_velocities(0,mid,wheels));
    assert(wheels.left_rad_s==0 && wheels.right_rad_s==0);
    assert(!manual_wheel_velocities(max,max+1,wheels));
    assert(wheels.left_rad_s==0 && wheels.right_rad_s==0);
    assert(sbus_to_wheel_velocity(mid+config::SBUS_DEADBAND+1)>0);
    // Forward throttle produces matching wheel directions after motor inversion.
    assert(manual_wheel_velocities(max,mid,wheels));
    can.sent.clear();
    assert(send_drive(can,wheels.left_rad_s,wheels.right_rad_s,failure));
    assert(value(can.sent[0])<0 && value(can.sent[2])>0);
}
''')
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                    "-Werror", "-Wno-unused-variable", "-I", str(tmp),
                    str(tmp / "drive_control.cpp"), str(tmp / "motor_interface.cpp"),
                    str(tmp / "test.cpp"), "-o", str(tmp / "test")], check=True)
    subprocess.run([str(tmp / "test")], check=True)
    # Compile the actual application entry point against the same host API stubs.
    shutil.copy(root / "src/main.cpp", tmp / "main.cpp")
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-I", str(tmp),
                    "-fsyntax-only", str(tmp / "main.cpp")], check=True)
print("Drive host tests passed")
