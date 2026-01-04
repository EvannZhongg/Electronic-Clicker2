#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h" 
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_random.h" // 【修复1】添加随机数头文件

// 【关键头文件】底层硬件操作
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "driver/periph_ctrl.h"

// BLE Includes
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "STRESS_TEST";

// *** 1. 引脚定义 ***
#define SEG_A_PIN   GPIO_NUM_4
#define SEG_B_PIN   GPIO_NUM_2
#define SEG_C_PIN   GPIO_NUM_7
#define SEG_D_PIN   GPIO_NUM_6
#define SEG_E_PIN   GPIO_NUM_5
#define SEG_F_PIN   GPIO_NUM_3
#define SEG_G_PIN   GPIO_NUM_8

#define DIG_1_PIN   GPIO_NUM_10 
#define DIG_2_PIN   GPIO_NUM_20 
#define DIG_3_PIN   GPIO_NUM_9

#define KEY_SWITCH_PIN  GPIO_NUM_0 
#define ZERO_SWITCH_PIN GPIO_NUM_1 

// *** 2. 测试控制变量 ***
static int32_t g_boot_count = 0; // 记录重启次数
// 【修复2】删除了 g_test_phase，因为逻辑中直接使用了 counter
static volatile int32_t g_display_value = 888; // 当前显示的数值
static bool    g_show_crash_count = true; // 是否显示崩溃计数

// BLE 相关
// 【修复3】删除了 g_ble_conn_handle，压力测试不需要维护连接句柄
static uint8_t g_ble_addr_type;
void start_ble_advertising(void);

// *** 3. 数码管驱动 ***
const uint8_t segment_map[11] = {
    (1 << 6),                                               // 0
    (1 << 0) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6),   // 1
    (1 << 2) | (1 << 5),                                    // 2
    (1 << 4) | (1 << 5),                                    // 3
    (1 << 0) | (1 << 3) | (1 << 4),                         // 4
    (1 << 1) | (1 << 4),                                    // 5
    (1 << 1),                                               // 6
    (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6),              // 7
    0,                                                      // 8 (全亮)
    (1 << 4),                                               // 9
    (1 << 0) | (1 << 1) | (1 << 2) | (1 << 7) // 'C' for Crash Count
};

void set_segments_gpio(uint8_t mapping) {
    gpio_set_level(SEG_A_PIN, (mapping >> 0) & 1);
    gpio_set_level(SEG_B_PIN, (mapping >> 1) & 1);
    gpio_set_level(SEG_C_PIN, (mapping >> 2) & 1);
    gpio_set_level(SEG_D_PIN, (mapping >> 3) & 1);
    gpio_set_level(SEG_E_PIN, (mapping >> 4) & 1);
    gpio_set_level(SEG_F_PIN, (mapping >> 5) & 1);
    gpio_set_level(SEG_G_PIN, (mapping >> 6) & 1);
}

void display_timer_callback(void* arg) {
    static int scan_index = 0; 
    
    // 消隐
    gpio_set_level(DIG_1_PIN, 1);
    gpio_set_level(DIG_2_PIN, 1);
    gpio_set_level(DIG_3_PIN, 1);

    uint8_t code = 0xFF; // 默认灭

    if (g_show_crash_count) {
        // 显示 C xx
        if (scan_index == 0) {
            code = segment_map[10]; // 'C'
        } else if (scan_index == 1) {
            code = segment_map[(g_boot_count / 10) % 10];
        } else {
            code = segment_map[g_boot_count % 10];
        }
    } else {
        // 显示数值
        int val = g_display_value;
        int digit = 0;
        if (scan_index == 0) digit = (val / 100) % 10;
        else if (scan_index == 1) digit = (val / 10) % 10;
        else digit = val % 10;
        
        code = segment_map[digit];
    }

    set_segments_gpio(code);

    if (scan_index == 0) gpio_set_level(DIG_1_PIN, 0);
    else if (scan_index == 1) gpio_set_level(DIG_2_PIN, 0);
    else if (scan_index == 2) gpio_set_level(DIG_3_PIN, 0);

    scan_index++;
    if (scan_index > 2) scan_index = 0;
}

void init_display_gpio(void) {
    gpio_reset_pin(DIG_2_PIN);
    gpio_reset_pin(DIG_3_PIN);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SEG_A_PIN) | (1ULL << SEG_B_PIN) | (1ULL << SEG_C_PIN) |
                        (1ULL << SEG_D_PIN) | (1ULL << SEG_E_PIN) | (1ULL << SEG_F_PIN) |
                        (1ULL << SEG_G_PIN) | (1ULL << DIG_1_PIN) | (1ULL << DIG_2_PIN) |
                        (1ULL << DIG_3_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// *** 4. NVS 崩溃记录 ***
void init_nvs_and_count(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_handle_t my_handle;
    ret = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (ret == ESP_OK) {
        int32_t restart_counter = 0;
        nvs_get_i32(my_handle, "restart_cnt", &restart_counter);
        
        g_boot_count = restart_counter + 1;
        nvs_set_i32(my_handle, "restart_cnt", g_boot_count);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        
        ESP_LOGE(TAG, "Device Reset Detected! Boot Count: %ld", g_boot_count);
    }
}

void reset_crash_count() {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_i32(my_handle, "restart_cnt", 0);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        g_boot_count = 0;
    }
}

// *** 5. BLE 占位 ***
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_DISCONNECT:
            start_ble_advertising();
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            start_ble_advertising(); 
            break;
    }
    return 0;
}

void start_ble_advertising(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"STRESS-TEST";
    fields.name_len = 11;
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    // 暴力广播间隔
    adv_params.itvl_min = 48; // 30ms
    adv_params.itvl_max = 64; // 40ms

    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

void ble_on_sync(void) {
    ble_hs_id_infer_auto(0, &g_ble_addr_type); 
    start_ble_advertising();
}

void ble_host_task(void *param) {
    nimble_port_run(); 
    nimble_port_freertos_deinit();
}

// *** 6. 压力测试主逻辑 ***
void stress_task(void *arg) {
    int counter = 0;
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << KEY_SWITCH_PIN) | (1ULL << ZERO_SWITCH_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    while (1) {
        // 清除计数检测
        if (gpio_get_level(ZERO_SWITCH_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            if (gpio_get_level(ZERO_SWITCH_PIN) == 0) {
                reset_crash_count();
                g_display_value = 0; g_show_crash_count = false;
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }

        // 状态机
        if (counter < 30) {
            g_show_crash_count = true; // 显示崩溃计数
        } else if (counter < 60) {
            g_show_crash_count = false;
            g_display_value = 888;     // 电流冲击
        } else if (counter < 90) {
            g_show_crash_count = false;
            g_display_value = (esp_random() % 1000); // 随机乱跳
        } else {
            counter = 0;
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // CPU 燃烧
        volatile float x = 1.5;
        for(int i=0; i<1000; i++) {
            x = x * 1.0001;
        }
    }
}

// *** 7. 主程序 ***
void app_main(void)
{
    // UART0 屏蔽逻辑
    periph_module_disable(PERIPH_UART0_MODULE);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, U0RXD_IN_IDX, false);
    gpio_reset_pin(GPIO_NUM_20);
    gpio_reset_pin(GPIO_NUM_21);

    init_nvs_and_count();

    init_display_gpio();
    esp_timer_handle_t display_timer;
    esp_timer_create_args_t timer_args = {
        .callback = &display_timer_callback,
        .name = "display_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &display_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(display_timer, 2000)); // 2ms 扫描

    nimble_port_init();
    ble_svc_gap_device_name_set("STRESS-TEST");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);

    xTaskCreate(stress_task, "stress", 4096, NULL, 5, NULL);
}
