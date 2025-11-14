#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_timer.h" // 【修改点】 包含 esp_timer.h 用于获取时间戳

// Button component
#include "iot_button.h"

// BLE (NimBLE) Includes
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "COUNTER_PROJECT";

// *** 1. 引脚定义 (根据你的方案) ***

// 7段数码管段位 (A-G)
#define SEG_A_PIN   GPIO_NUM_3
#define SEG_B_PIN   GPIO_NUM_7
#define SEG_C_PIN   GPIO_NUM_6
#define SEG_D_PIN   GPIO_NUM_4
#define SEG_E_PIN   GPIO_NUM_2
#define SEG_F_PIN   GPIO_NUM_5
#define SEG_G_PIN   GPIO_NUM_8

// 3位共阳数码管位选 (DIG1-3)
#define DIG_1_PIN   GPIO_NUM_10 // 百位
#define DIG_2_PIN   GPIO_NUM_19 // 十位
#define DIG_3_PIN   GPIO_NUM_18 // 个位

// 输入引脚
#define KEY_SWITCH_PIN  GPIO_NUM_0 // 键轴按键
#define ZERO_SWITCH_PIN GPIO_NUM_1 // 置零按键

// 按键句柄
static button_handle_t g_key_button_handle = NULL;
static button_handle_t g_zero_button_handle = NULL;

// *** 2. 全局变量 ***

// 计数器 (volatile 保证多任务安全访问，支持-99到999范围)
static volatile int32_t g_counter = 0;


// 【修改点】 BLE 通知的数据结构
// 我们将发送一个结构体，而不是一个单独的int
// 1字节对齐，确保在不同平台（ESP32/PC）上内存布局一致
#pragma pack(push, 1)
typedef struct {
    int8_t event_type;      // 事件类型: 1 (加), -1 (减), 0 (清零)
    uint32_t timestamp_ms;  // 事件发生时的时间戳 (毫秒)
    int32_t current_total;  // 事件发生后的当前总数
} counter_event_t;
#pragma pack(pop)

// 用于BLE通知的事件数据
static volatile counter_event_t g_last_event;
// 计数器事件更新标志，通知BLE任务
static volatile bool g_event_updated = false;


// 用于BLE连接
static uint16_t g_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;

// BLE 句柄和地址类型变量
static uint16_t g_counter_val_handle;
static uint8_t g_ble_addr_type;

// *** 声明函数原型 ***
void start_ble_advertising(void);


// *** 3. 数码管显示任务 ***

// 7段数码管编码 (0-9)
// 编码格式：g,f,e,d,c,b,a (bit 6-0)，1=熄灭，0=点亮
const uint8_t segment_map[10] = {
    (1 << 6),                               // 0: a,b,c,d,e,f (熄灭g)
    (1 << 0) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6), // 1: b,c (熄灭a,d,e,f,g)
    (1 << 2) | (1 << 5),                    // 2: a,b,d,e,g (熄灭c,f)
    (1 << 4) | (1 << 5),                    // 3: a,b,c,d,g (熄灭e,f)
    (1 << 0) | (1 << 3) | (1 << 4),         // 4: b,c,f,g (熄灭a,d,e)
    (1 << 1) | (1 << 4),                    // 5: a,c,d,f,g (熄灭b,e)
    (1 << 1),                               // 6: a,c,d,e,f,g (熄灭b)
    (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6), // 7: a,b,c (熄灭d,e,f,g)
    0,                                      // 8: a,b,c,d,e,f,g (全部点亮)
    (1 << 4)                                // 9: a,b,c,d,f,g (熄灭e，修复：a段应该点亮)
};

// 根据数字设置7个段位引脚
void set_segments(uint8_t num) {
    if (num > 9) num = 0; // 仅显示0-9
    uint8_t mapping = segment_map[num];

    // 1 = 熄灭, 0 = 点亮
    gpio_set_level(SEG_A_PIN, (mapping >> 0) & 1);
    gpio_set_level(SEG_B_PIN, (mapping >> 1) & 1);
    gpio_set_level(SEG_C_PIN, (mapping >> 2) & 1);
    gpio_set_level(SEG_D_PIN, (mapping >> 3) & 1);
    gpio_set_level(SEG_E_PIN, (mapping >> 4) & 1);
    gpio_set_level(SEG_F_PIN, (mapping >> 5) & 1);
    gpio_set_level(SEG_G_PIN, (mapping >> 6) & 1);
}

// 显示负号（只点亮G段，其他段熄灭）
void set_minus_sign(void) {
    // 1 = 熄灭, 0 = 点亮
    // 负号：只点亮G段（bit 6），其他段全部熄灭
    gpio_set_level(SEG_A_PIN, 1);  // 熄灭
    gpio_set_level(SEG_B_PIN, 1);  // 熄灭
    gpio_set_level(SEG_C_PIN, 1);  // 熄灭
    gpio_set_level(SEG_D_PIN, 1);  // 熄灭
    gpio_set_level(SEG_E_PIN, 1);  // 熄灭
    gpio_set_level(SEG_F_PIN, 1);  // 熄灭
    gpio_set_level(SEG_G_PIN, 0);  // 点亮G段（负号）
}

// 初始化数码管所有GPIO
void init_display_gpio(void) {
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

// 数码管动态扫描任务
void display_task(void *pvParameters) {
    int32_t count;
    uint8_t digit1, digit2, digit3;
    bool is_negative = false;
    uint8_t abs_value;

    // 初始关闭所有位选 (共阳，高电平关闭)
    gpio_set_level(DIG_1_PIN, 1);
    gpio_set_level(DIG_2_PIN, 1);
    gpio_set_level(DIG_3_PIN, 1);

    while (1) {
        count = g_counter;
        is_negative = (count < 0);
        
        // 获取绝对值用于显示
        if (is_negative) {
            abs_value = (uint8_t)(-count);  // 转换为正数
        } else {
            abs_value = (uint8_t)count;
        }

        // 计算各位数字
        digit1 = abs_value % 10;        // 个位
        digit2 = (abs_value / 10) % 10;  // 十位
        digit3 = (abs_value / 100) % 10; // 百位

        if (is_negative) {
            // 负数显示逻辑
            if (abs_value >= 10) {
                // 两位数或三位数负数（-10到-99），负号在DIG1
                // --- 显示负号 (DIG1) ---
                set_minus_sign();
                gpio_set_level(DIG_1_PIN, 0); // 打开百位显示负号
                vTaskDelay(pdMS_TO_TICKS(6));  // 6ms显示时间
                gpio_set_level(DIG_1_PIN, 1); // 关闭
                vTaskDelay(pdMS_TO_TICKS(1));  // 位切换间隔

                // --- 显示十位 (DIG2) ---
                set_segments(digit2);
                gpio_set_level(DIG_2_PIN, 0); // 打开十位
                vTaskDelay(pdMS_TO_TICKS(6));  // 6ms显示时间
                gpio_set_level(DIG_2_PIN, 1); // 关闭十位
                vTaskDelay(pdMS_TO_TICKS(1));  // 位切换间隔

                // --- 显示个位 (DIG3) ---
                set_segments(digit1);
                gpio_set_level(DIG_3_PIN, 0); // 打开个位
                vTaskDelay(pdMS_TO_TICKS(6));  // 6ms显示时间
                gpio_set_level(DIG_3_PIN, 1); // 关闭个位
            } else {
                // 一位数负数（-1到-9），负号在DIG2
                // --- 显示百位 (DIG1) ---（空白，不显示）
                // 跳过DIG1，不显示任何内容

                // --- 显示负号 (DIG2) ---
                set_minus_sign();
                gpio_set_level(DIG_2_PIN, 0); // 打开十位显示负号
                vTaskDelay(pdMS_TO_TICKS(6));  // 6ms显示时间
                gpio_set_level(DIG_2_PIN, 1); // 关闭
                vTaskDelay(pdMS_TO_TICKS(1));  // 位切换间隔

                // --- 显示个位 (DIG3) ---
                set_segments(digit1);
                gpio_set_level(DIG_3_PIN, 0); // 打开个位
                vTaskDelay(pdMS_TO_TICKS(6));  // 6ms显示时间
                gpio_set_level(DIG_3_PIN, 1); // 关闭个位
            }
        } else {
            // 正数显示逻辑（0-999）
            // --- 显示百位 (DIG1) ---
            set_segments(digit3);
            gpio_set_level(DIG_1_PIN, 0); // 打开百位
            vTaskDelay(pdMS_TO_TICKS(6));  // 6ms显示时间
            gpio_set_level(DIG_1_PIN, 1); // 关闭百位
            vTaskDelay(pdMS_TO_TICKS(1));  // 位切换间隔

            // --- 显示十位 (DIG2) ---
            set_segments(digit2);
            gpio_set_level(DIG_2_PIN, 0); // 打开十位
            vTaskDelay(pdMS_TO_TICKS(6));  // 6ms显示时间
            gpio_set_level(DIG_2_PIN, 1); // 关闭十位
            vTaskDelay(pdMS_TO_TICKS(1));  // 位切换间隔

            // --- 显示个位 (DIG3) ---
            set_segments(digit1);
            gpio_set_level(DIG_3_PIN, 0); // 打开个位
            vTaskDelay(pdMS_TO_TICKS(6));  // 6ms显示时间
            gpio_set_level(DIG_3_PIN, 1); // 关闭个位
        }
        
        // 【【【 关键修复 】】】
        // 您的 vTaskDelay(pdMS_TO_TICKS(6)) 和 (1) 都会被编译为 0 Tick 延迟
        // 因为您的系统 HZ 是 100，1 Tick = 10ms。
        // 必须添加一个大于 0 Tick 的延迟来让出CPU，否则IDLE任务会"饿死"
        vTaskDelay(1); // 至少让出1个 FreeRTOS Tick (即 10ms)
    }
}


// *** 4. 按键处理 (使用 button 组件) ***

// 键轴按键按下回调（单击事件）
static void key_button_single_click_cb(void *arg, void *usr_data) {
    // 计数加一，限制范围到999
    if (g_counter < 999) {
        g_counter++;
    }
    
    // 【修改点】 填充事件结构体
    g_last_event.event_type = 1; // +1 事件
    g_last_event.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000); // 毫秒时间戳
    g_last_event.current_total = g_counter;
    g_event_updated = true; // 设置事件更新标志
    
    ESP_LOGI(TAG, "按键按下! 计数: %ld", g_counter);
}

// 置零按键单击回调（轻触减一）
static void zero_button_single_click_cb(void *arg, void *usr_data) {
    // 计数减一，允许负数，限制范围到-99
    if (g_counter > -99) {
        g_counter--;
    }

    // 【修改点】 填充事件结构体
    g_last_event.event_type = -1; // -1 事件
    g_last_event.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000); // 毫秒时间戳
    g_last_event.current_total = g_counter;
    g_event_updated = true; // 设置事件更新标志

    ESP_LOGI(TAG, "置零按键轻触，计数减一: %ld", g_counter);
}

// 置零按键长按回调（5秒长按清零）
static void zero_button_long_press_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "置零按键长按5秒，计数清零");
    g_counter = 0;

    // 【修改点】 填充事件结构体
    g_last_event.event_type = 0; // 0 (清零) 事件
    g_last_event.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000); // 毫秒时间戳
    g_last_event.current_total = g_counter;
    g_event_updated = true; // 设置事件更新标志
}

// 初始化按键（使用 button 组件）
void init_inputs(void) {
    esp_err_t ret;
    
    // 1. 初始化键轴按键 (GPIO0)
    button_config_t key_button_config = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = 0,  // 不需要长按功能
        .short_press_time = 10,  // 短按时间10ms（用于防抖）
        .gpio_button_config = {
            .gpio_num = KEY_SWITCH_PIN,
            .active_level = 0,  // 低电平有效（按下时为低电平）
        },
    };
    
    g_key_button_handle = iot_button_create(&key_button_config);
    if (g_key_button_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create key button");
        return;
    }
    
    // 注册单击事件回调
    ret = iot_button_register_cb(g_key_button_handle, BUTTON_SINGLE_CLICK, key_button_single_click_cb, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register key button callback: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Key button (GPIO%d) initialized successfully", KEY_SWITCH_PIN);
    }
    
    // 2. 初始化置零按键 (GPIO1)
    button_config_t zero_button_config = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = 5000,  // 5秒长按
        .short_press_time = 10,  // 短按时间10ms（用于防抖）
        .gpio_button_config = {
            .gpio_num = ZERO_SWITCH_PIN,
            .active_level = 0,  // 低电平有效（按下时为低电平）
        },
    };
    
    g_zero_button_handle = iot_button_create(&zero_button_config);
    if (g_zero_button_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create zero button");
        return;
    }
    
    // 注册单击事件回调（轻触减一）
    ret = iot_button_register_cb(g_zero_button_handle, BUTTON_SINGLE_CLICK, zero_button_single_click_cb, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register zero button single click callback: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Zero button (GPIO%d) single click callback registered (decrement)", ZERO_SWITCH_PIN);
    }
    
    // 注册长按事件回调（5秒长按清零）
    ret = iot_button_register_cb(g_zero_button_handle, BUTTON_LONG_PRESS_START, zero_button_long_press_cb, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register zero button long press callback: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Zero button (GPIO%d) initialized successfully: single click=decrement, long press 5s=reset", ZERO_SWITCH_PIN);
    }
}


// *** 5. BLE (NimBLE) 通信 ***

// 定义 UUID
static const ble_uuid128_t gatt_svc_uuid =
    BLE_UUID128_INIT(0x28, 0x91, 0xae, 0x8d, 0x3d, 0x45, 0x4f, 0xde,
                     0x81, 0x4a, 0x51, 0x69, 0xd0, 0x18, 0x50, 0x01);
static const ble_uuid128_t gatt_chr_uuid =
    BLE_UUID128_INIT(0x28, 0x91, 0xae, 0x8d, 0x3d, 0x45, 0x4f, 0xde,
                     0x81, 0x4a, 0x51, 0x69, 0xd0, 0x18, 0x50, 0x02);

// GATT 事件处理
static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // 【修改点】
    // 当客户端 *读取* (READ) 特征时，我们只发送当前的 *总分* (int32_t)
    // 当我们 *通知* (NOTIFY) 时，我们会发送完整的 *事件* (counter_event_t)
    // 这种设计是允许的，并且很常用：读取=获取当前状态，通知=获取事件流

    // 客户端读取计数值时调用
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // 创建 g_counter 的本地副本以移除 volatile 限定符
        int32_t count_copy = g_counter;
        int rc = os_mbuf_append(ctxt->om, &count_copy, sizeof(count_copy));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    
    // 客户端写入 CCCD (启用/禁用 Notify)
    // 这个回调也会处理对CCCD的写入，这是启用通知所必需的，所以保留它
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
         ESP_LOGI(TAG, "客户端写入特性（可能是CCCD）");
    }

    return 0;
}

// 定义GATT服务
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &gatt_chr_uuid.u,
                .access_cb = gatt_svc_access,
                // 保持 READ | NOTIFY
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &g_counter_val_handle,
            },
            {0} // 结束
        },
    },
    {0} // 结束
};

// 【修改点】 重命名并修改此函数以发送事件结构体
void ble_notify_event(void) {
    // 协议栈会自动检查谁订阅了通知。我们只需要检查连接句柄是否有效。
    if (g_ble_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        
        // 创建 g_last_event 的本地副本以移除 volatile
        counter_event_t event_copy = g_last_event;
        
        // 从我们的事件结构体创建 mbuf
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&event_copy, sizeof(event_copy));
        
        if (om) {
            // NimBLE 会自动只发送给已订阅此特征的客户端
            int rc = ble_gatts_notify_custom(g_ble_conn_handle, g_counter_val_handle, om);
            if (rc == 0) {
                 ESP_LOGI(TAG, "成功发送通知: type=%d, ts=%lu, total=%ld",
                       event_copy.event_type, event_copy.timestamp_ms, event_copy.current_total);
            } else {
                 ESP_LOGE(TAG, "发送通知失败, rc=%d", rc);
            }
        }
    }
}

// BLE GAP 事件处理
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
        ESP_LOGI(TAG, "BLE Advertising Complete");
        start_ble_advertising(); // 广播可能超时，重启
        break;
    }
    return 0;
}

// 启动 BLE 广播
void start_ble_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE Advertising started");
    } else {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %d", rc);
    }
}

// BLE 协议栈同步回调
void ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, &g_ble_addr_type); 
    start_ble_advertising();
}

// BLE 主任务
void ble_host_task(void *param)
{
    nimble_port_run(); // 此函数不会返回
    nimble_port_freertos_deinit();
}

// *** 6. 主程序 (app_main) ***

void app_main(void)
{
    esp_err_t ret;

    // 初始化 NVS (BLE 必需)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1. 初始化硬件
    init_display_gpio();
    init_inputs();

    // 2. 启动数码管显示任务
    xTaskCreate(display_task, "display_task", 2048, NULL, 1, NULL);

    // 3. 初始化 BLE
    nimble_port_init();
    
    // 获取MAC地址后4位作为设备标识，支持多设备识别
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char device_name[32];
    snprintf(device_name, sizeof(device_name), "Counter-%02X%02X", mac[4], mac[5]);
    ble_svc_gap_device_name_set(device_name); // 设置BLE设备名称（使用MAC地址后4位）
    ESP_LOGI(TAG, "BLE Device Name: %s", device_name);
    
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "ESP32 Counter Initialized");
    ESP_LOGI(TAG, "Current counter value: %ld", g_counter);

    // 4. 主循环
    while (1) {
        
        // 【修改点】 检查事件标志
        if (g_event_updated) {
            ble_notify_event(); // 发送事件通知
            g_event_updated = false;
        }

        // 主循环延迟，让出CPU给其他任务
        vTaskDelay(pdMS_TO_TICKS(50)); // 可以缩短轮询间隔以便更快地发送通知
    }
}