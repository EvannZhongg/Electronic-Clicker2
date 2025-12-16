#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

// 【关键头文件】用于底层硬件操作
#include "esp_rom_gpio.h"       // GPIO 矩阵操作
#include "soc/gpio_sig_map.h"   // 信号索引定义
#include "driver/periph_ctrl.h" // 外设时钟控制

// BLE (NimBLE) Includes
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "COUNTER_PROJECT";

// *** 1. 引脚定义 ***

// 7段数码管段位 (A-G)
#define SEG_A_PIN   GPIO_NUM_4
#define SEG_B_PIN   GPIO_NUM_2
#define SEG_C_PIN   GPIO_NUM_7
#define SEG_D_PIN   GPIO_NUM_6
#define SEG_E_PIN   GPIO_NUM_5
#define SEG_F_PIN   GPIO_NUM_3
#define SEG_G_PIN   GPIO_NUM_8


#define DIG_1_PIN   GPIO_NUM_10 
#define DIG_2_PIN   GPIO_NUM_20 // 注意：IO20 需要在 app_main 中禁用 UART0，你的代码已包含此逻辑
#define DIG_3_PIN   GPIO_NUM_9

// 输入引脚
#define KEY_SWITCH_PIN  GPIO_NUM_0 // 键轴按键
#define ZERO_SWITCH_PIN GPIO_NUM_1 // 置零按键

// *** 2. 全局变量与数据结构 ***

static volatile int32_t g_counter = 0;

static volatile int32_t g_total_plus_counts = 0;
static volatile int32_t g_total_minus_counts = 0;

static char g_device_name[32] = "Counter-Init";

#pragma pack(push, 1)
typedef struct {
    int32_t current_total;
    int8_t  event_type;
    int32_t total_plus;
    int32_t total_minus;
    uint32_t timestamp_ms;
} counter_event_t;
#pragma pack(pop)

static volatile counter_event_t g_last_event;
static volatile bool g_event_updated = false;

static uint16_t g_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_counter_val_handle;
static uint8_t g_ble_addr_type;

void start_ble_advertising(void);

// *** 3. 数码管定时器扫描 ***

const uint8_t segment_map[10] = {
    (1 << 6),                               // 0
    (1 << 0) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6), // 1
    (1 << 2) | (1 << 5),                    // 2
    (1 << 4) | (1 << 5),                    // 3
    (1 << 0) | (1 << 3) | (1 << 4),         // 4
    (1 << 1) | (1 << 4),                    // 5
    (1 << 1),                               // 6
    (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6), // 7
    0,                                      // 8
    (1 << 4)                                // 9
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

    int32_t count = g_counter;
    uint32_t abs_val;
    bool is_neg = false;
    if (count < 0) {
        is_neg = true;
        abs_val = (uint32_t)(-count);
    } else {
        abs_val = (uint32_t)count;
    }

    uint8_t digit_val = 0;
    bool show_minus = false; 
    bool show_blank = false; 

    switch (scan_index) {
        case 0: 
            if (is_neg && abs_val >= 10) show_minus = true; 
            else if (!is_neg) digit_val = (abs_val / 100) % 10;
            else show_blank = true; 
            break;
        case 1: 
            if (is_neg && abs_val < 10) show_minus = true; 
            else digit_val = (abs_val / 10) % 10;
            break;
        case 2: 
            digit_val = abs_val % 10;
            break;
    }

    if (show_blank) {
        set_segments_gpio(0xFF); 
    } else if (show_minus) {
        gpio_set_level(SEG_A_PIN, 1); gpio_set_level(SEG_B_PIN, 1);
        gpio_set_level(SEG_C_PIN, 1); gpio_set_level(SEG_D_PIN, 1);
        gpio_set_level(SEG_E_PIN, 1); gpio_set_level(SEG_F_PIN, 1);
        gpio_set_level(SEG_G_PIN, 0);
    } else {
        if (digit_val > 9) digit_val = 0;
        set_segments_gpio(segment_map[digit_val]);
    }

    if (!show_blank) {
        if (scan_index == 0) gpio_set_level(DIG_1_PIN, 0);
        else if (scan_index == 1) gpio_set_level(DIG_2_PIN, 0);
        else if (scan_index == 2) gpio_set_level(DIG_3_PIN, 0);
    }

    scan_index++;
    if (scan_index > 2) scan_index = 0;
}

void init_display_gpio(void) {
    // 即使在 app_main 断开了矩阵，这里通过 gpio_config 再次确认引脚为输出模式
    // 这会将 IO MUX 切换到 GPIO 功能
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

// *** 4. 按键处理 ***

void update_event(int8_t type) {
    g_last_event.current_total = g_counter;
    g_last_event.event_type = type;
    g_last_event.total_plus = g_total_plus_counts;
    g_last_event.total_minus = g_total_minus_counts;
    g_last_event.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    g_event_updated = true;
}

void handle_key_action(void) {
    if (g_counter >= 999) g_counter = -99;
    else g_counter++;
    g_total_plus_counts++;
    update_event(1);
}

void handle_zero_click(void) {
    if (g_counter <= -99) g_counter = 999;
    else g_counter--;
    g_total_minus_counts++;
    update_event(-1);
}

void handle_zero_long_press(void) {
    g_counter = 0;
    g_total_plus_counts = 0;  
    g_total_minus_counts = 0; 
    update_event(0); 
    ESP_LOGI(TAG, "Counter Reset");
}

#define BTN_DEBOUNCE_TICKS  3
#define LONG_PRESS_TICKS    300 

void button_poll_task(void *arg) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << KEY_SWITCH_PIN) | (1ULL << ZERO_SWITCH_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    int key_stable = 1; int key_cnt = 0;
    int zero_stable = 1; int zero_cnt = 0;
    int zero_press_time = 0; bool zero_handled = false;

    while (1) {
        int key_raw = gpio_get_level(KEY_SWITCH_PIN);
        if (key_raw != key_stable) {
            if (++key_cnt >= BTN_DEBOUNCE_TICKS) {
                key_stable = key_raw;
                key_cnt = 0;
                if (key_stable == 0) handle_key_action();
            }
        } else key_cnt = 0;

        int zero_raw = gpio_get_level(ZERO_SWITCH_PIN);
        if (zero_raw != zero_stable) {
            if (++zero_cnt >= BTN_DEBOUNCE_TICKS) {
                zero_stable = zero_raw;
                zero_cnt = 0;
                if (zero_stable == 0) { zero_press_time = 0; zero_handled = false; }
                else if (!zero_handled) handle_zero_click();
            }
        } else {
            zero_cnt = 0;
            if (zero_stable == 0) {
                if (++zero_press_time >= LONG_PRESS_TICKS && !zero_handled) {
                    handle_zero_long_press();
                    zero_handled = true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}


// *** 5. BLE (NimBLE) ***

static const ble_uuid128_t gatt_svc_uuid =
    BLE_UUID128_INIT(0x28, 0x91, 0xae, 0x8d, 0x3d, 0x45, 0x4f, 0xde,
                     0x81, 0x4a, 0x51, 0x69, 0xd0, 0x18, 0x50, 0x01);
static const ble_uuid128_t gatt_chr_uuid =
    BLE_UUID128_INIT(0x28, 0x91, 0xae, 0x8d, 0x3d, 0x45, 0x4f, 0xde,
                     0x81, 0x4a, 0x51, 0x69, 0xd0, 0x18, 0x50, 0x02);

void handle_zero_long_press(void); 

static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // 1. 处理读取 (保持原样)
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // ... (保持你原有的读取逻辑) ...
        counter_event_t evt;
        evt.current_total = g_counter;
        evt.event_type = 0; // Read 操作默认类型
        evt.total_plus = g_total_plus_counts;
        evt.total_minus = g_total_minus_counts;
        evt.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
        
        int rc = os_mbuf_append(ctxt->om, &evt, sizeof(evt));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    // 2. 【新增】处理写入 (PC 发送重置指令)
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        // 从 buffer 中读取数据
        uint8_t data[1];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        
        if (len > 0) {
            int rc = ble_hs_mbuf_to_flat(ctxt->om, data, sizeof(data), NULL);
            if (rc != 0) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

            // 约定：如果收到 0x01，则执行重置
            if (data[0] == 0x01) {
                ESP_LOGI(TAG, "Received Reset Command from PC");
                // 复用已有的长按清零逻辑，它会重置变量并调用 update_event(0)
                handle_zero_long_press(); 
            }
        }
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &gatt_chr_uuid.u,
                .access_cb = gatt_svc_access,
                // 【修改】增加 BLE_GATT_CHR_F_WRITE 权限
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_WRITE, 
                .val_handle = &g_counter_val_handle,
            },
            {0}
        },
    },
    {0}
};

void ble_notify_event(void) {
    if (g_ble_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        counter_event_t event_copy = g_last_event;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&event_copy, sizeof(event_copy));
        if (om) {
            int rc = ble_gatts_notify_custom(g_ble_conn_handle, g_counter_val_handle, om);
            // 【重要修复】如果发送失败，必须手动释放 mbuf，否则会造成内存泄漏
            if (rc != 0) {
                os_mbuf_free_chain(om);
                ESP_LOGW(TAG, "Notify failed: %d", rc);
            }
        }
    }
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE Connected");
        g_ble_conn_handle = event->connect.conn_handle;
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE Disconnected");
        g_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        start_ble_advertising();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_ble_advertising(); 
        break;
    }
    return 0;
}

void start_ble_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) ESP_LOGE(TAG, "Adv fields error: %d", rc);

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) ESP_LOGE(TAG, "Adv start error: %d", rc);
}

void ble_on_sync(void) {
    ble_hs_id_infer_auto(0, &g_ble_addr_type); 
    start_ble_advertising();
}

void ble_host_task(void *param) {
    nimble_port_run(); 
    nimble_port_freertos_deinit();
}

// *** 6. 主程序 ***

void app_main(void)
{
    // ================================================================
    // 【终极修复】物理切断 UART0 的所有输入源和时钟
    // 这是一个“核选项”，确保无论之前的状态如何，UART0 都彻底死掉。
    // ================================================================
    
    // 1. 强制关闭 UART0 外设时钟（切断电源）
    periph_module_disable(PERIPH_UART0_MODULE);
    
    // 2. 修改 GPIO 交换矩阵：强行将 UART0 的 RX 输入信号连接到一个常量“1”（高电平）
    // 这样，无论 GPIO 20 怎么翻转，UART0 内部只看到一条死一般平静的直线，永远不会触发中断。
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, U0RXD_IN_IDX, false);
    
    // 3. 物理复位引脚
    gpio_reset_pin(GPIO_NUM_20);
    gpio_reset_pin(GPIO_NUM_21);

    // ================================================================

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_display_gpio();
    
    xTaskCreate(button_poll_task, "btn_poll", 4096, NULL, 1, NULL);

    nimble_port_init();
    
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    snprintf(g_device_name, sizeof(g_device_name), "Counter-%02X%02X", mac[4], mac[5]);
    ble_svc_gap_device_name_set(g_device_name); 
    
    ESP_LOGI(TAG, "BLE Name Set: %s", g_device_name);
    
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
    
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_timer_handle_t display_timer;
    esp_timer_create_args_t timer_args = {
        .callback = &display_timer_callback,
        .name = "display_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &display_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(display_timer, 5000)); 

    ESP_LOGI(TAG, "System Running on new pins (20/21)");
    
    while (1) {
        if (g_event_updated) {
            ble_notify_event(); 
            g_event_updated = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}