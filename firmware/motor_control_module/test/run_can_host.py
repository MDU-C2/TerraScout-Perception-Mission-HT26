"""Test actual CAN slot management against a deterministic mock TWAI driver."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="terrascout-can-") as directory:
    tmp = Path(directory)
    shutil.copy(root / "include/can.hpp", tmp / "can.hpp")
    shutil.copy(root / "src/can.cpp", tmp / "can.cpp")
    (tmp / "mock.hpp").write_text(r'''#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
using gpio_num_t=int; constexpr int GPIO_NUM_NC=-1;
using esp_err_t=int;
constexpr int ESP_OK=0, ESP_ERR_INVALID_STATE=1, ESP_ERR_INVALID_ARG=2,
 ESP_ERR_TIMEOUT=3, ESP_ERR_NO_MEM=4;
inline const char* esp_err_to_name(int){return "stub";}
#define IRAM_ATTR
template<class... Args> inline void mock_log(Args... args){((void)args,...);}
#define ESP_LOGI(...) mock_log(__VA_ARGS__)
#define ESP_LOGE(...) mock_log(__VA_ARGS__)
using QueueHandle_t=void*; using SemaphoreHandle_t=void*;
using BaseType_t=int; constexpr int pdTRUE=1,pdFALSE=0;
inline int pdMS_TO_TICKS(int n){return n;}
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) do{(void)(p);}while(0)
#define portEXIT_CRITICAL(p) do{(void)(p);}while(0)
#define portENTER_CRITICAL_ISR(p) portENTER_CRITICAL(p)
#define portEXIT_CRITICAL_ISR(p) portEXIT_CRITICAL(p)
inline void* xQueueCreate(size_t,size_t){return reinterpret_cast<void*>(1);}
inline void* xSemaphoreCreateMutex(){return reinterpret_cast<void*>(1);}
inline int xSemaphoreTake(void*,int){return pdTRUE;}
inline void xSemaphoreGive(void*){}
inline void vQueueDelete(void*){}
inline void vSemaphoreDelete(void*){}
inline int xQueueReceive(void*,void*,int){return pdFALSE;}
inline int xQueueSendFromISR(void*,void*,int*){return pdTRUE;}
using twai_node_handle_t=void*;
struct twai_frame_t {struct {uint32_t id{}; bool ide{},rtr{}; uint8_t dlc{};}header;
 uint8_t* buffer{};size_t buffer_len{};};
struct twai_rx_done_event_data_t{};
struct twai_tx_done_event_data_t{const twai_frame_t* done_tx_frame; bool is_tx_success=true;};
struct twai_node_status_t {int state=0; uint16_t tx_error_count=0,rx_error_count=0;uint32_t tx_queue_remaining=0;};
struct twai_node_record_t {uint32_t bus_err_num=0;};
inline int twai_node_get_info(void*,twai_node_status_t*,twai_node_record_t*){return 0;}
struct twai_state_change_event_data_t{int new_sta;};
constexpr int TWAI_ERROR_BUS_OFF=1;
struct twai_onchip_node_config_t {
 struct {int tx{},rx{},quanta_clk_out{},bus_off_indicator{};}io_cfg;
 struct {uint32_t bitrate{};}bit_timing;
 uint32_t tx_queue_depth{};int fail_retry_cnt{};
 struct {bool enable_self_test{};}flags;
};
struct twai_event_callbacks_t {
 bool(*on_rx_done)(void*,const twai_rx_done_event_data_t*,void*){};
 bool(*on_state_change)(void*,const twai_state_change_event_data_t*,void*){};
 bool(*on_tx_done)(void*,const twai_tx_done_event_data_t*,void*){};
};
inline twai_event_callbacks_t callbacks;inline void* context;
inline std::vector<const twai_frame_t*> pending;
inline int last_timeout=-1;
inline int twai_new_node_onchip(const twai_onchip_node_config_t*,void**node){*node=reinterpret_cast<void*>(1);return 0;}
inline int twai_node_register_event_callbacks(void*,const twai_event_callbacks_t*c,void*ctx){callbacks=*c;context=ctx;return 0;}
inline int twai_node_enable(void*){return 0;}
inline int twai_node_disable(void*){return 0;}
inline int twai_node_delete(void*){pending.clear();return 0;}
inline int twai_node_transmit(void*,const twai_frame_t*f,int timeout){
 last_timeout=timeout;
 if(pending.size()>=2)return ESP_ERR_TIMEOUT;
 pending.push_back(f);return 0;
}
inline int twai_node_transmit_wait_all_done(void*,int){return 0;}
inline int twai_node_receive_from_isr(void*,twai_frame_t*){return 1;}
inline int twai_node_recover(void*){return 0;}
''')
    for name in ("driver/gpio.h", "esp_err.h", "esp_twai.h", "esp_twai_onchip.h",
                 "freertos/FreeRTOS.h", "freertos/queue.h", "freertos/semphr.h",
                 "esp_attr.h", "esp_log.h"):
        target = tmp / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text('#include "mock.hpp"\n')
    (tmp / "test.cpp").write_text(r'''
#include "can.hpp"
#include <cassert>
int main(){
 Can can; assert(can.begin(5,4,1000000,16,2)==ESP_OK);
 uint8_t a[]={1,2,3,4}, b[]={5,6,7,8}, c[]={9,10,11,12};
 assert(can.send(0x301,a,4,true,0)==ESP_OK);
 assert(can.send(0x302,b,4,true,0)==ESP_OK);
 const auto* first=pending[0]; const auto* second=pending[1];
 for(int i=0;i<20;++i) {
  assert(can.send(0x303,c,4,true,0)==ESP_ERR_TIMEOUT);
  assert(first->header.id==0x301 && first->buffer[0]==1);
  assert(second->header.id==0x302 && second->buffer[0]==5);
 }
 // Complete the second frame first: only its slot becomes reusable.
 twai_tx_done_event_data_t done{second}; pending.erase(pending.begin()+1);
 callbacks.on_tx_done(nullptr,&done,context);
 assert(can.send(0x303,c,4,true,0)==ESP_OK);
 assert(first->header.id==0x301 && first->buffer[0]==1);
 assert(last_timeout==0);
 assert(can.send(0x20000000,c,4,true,0)==ESP_ERR_INVALID_ARG);
 assert(can.send(0x800,c,4,false,0)==ESP_ERR_INVALID_ARG);
 assert(can.send(0x301,nullptr,4,true,0)==ESP_ERR_INVALID_ARG);
 assert(can.send(0x301,c,9,true,0)==ESP_ERR_INVALID_ARG);
}
''')
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                    "-Werror", "-Wno-unused-variable", "-I", str(tmp),
                    str(tmp / "can.cpp"), str(tmp / "test.cpp"),
                    "-o", str(tmp / "test")], check=True)
    subprocess.run([str(tmp / "test")], check=True)
print("CAN host tests passed")
