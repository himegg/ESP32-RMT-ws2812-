/**
 * ============================================================================
 * ws2812.c — WS2812B 可寻址 RGB LED 驱动实现 (ESP32-S3 + RMT)
 * ============================================================================
 *
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │                              目    录                                    │
 * ├──────────────────────────────────────────────────────────────────────────┤
 * │ 一、协议原理    — WS2812B 的 NRZ 编码与 RMT 硬件实现                      │
 * │ 二、架构设计    — 静态槽位数组、句柄缓存、零动态分配                       │
 * │ 三、数据流      — 从 API 调用到 GPIO 波形的完整路径                       │
 * │ 四、关键 Bug 复盘 — static 变量零初始化陷阱（2026-05 真实踩坑）           │
 * │ 五、错误处理    — 回滚策略与 ESP-IDF 惯用模式                            │
 * │ 六、可调参数    — 时序微调、通道数、内存块配置                            │
 * └──────────────────────────────────────────────────────────────────────────┘
 *
 * ============================================================================
 * 一、协议原理
 * ============================================================================
 *
 *   WS2812B 使用一根数据线（单线协议），通过不同长度的高电平脉冲表示 0 和 1。
 *   这叫做 NRZ（Non-Return-to-Zero，不归零）编码。
 *
 *   波形示意：
 *
 *     "0" 码:   ─┐‾‾┌──────────    高 0.40us + 低 0.85us = 1.25us
 *     "1" 码:   ─┐‾‾‾‾‾┌──────    高 0.80us + 低 0.45us = 1.25us
 *     RESET:    ───────────────    低电平持续 50us 以上（帧结束标志）
 *
 *   一帧完整数据 = 24 bits（G[7:0] → R[7:0] → B[7:0]，每个字节 MSB 先发）
 *   帧与帧之间必须用 RESET 码（>50us 低电平）隔开。
 *
 *   为什么用 RMT 而不是 GPIO 模拟（bit-banging）？
 *   — GPIO 软件翻转受中断影响，纳秒级时序无法保证
 *   — RMT 硬件自动产生信号，不受 CPU 负载影响
 *   — rmt_bytes_encoder 自动完成"字节→RMT符号"转换，大大简化代码
 *
 *   时钟与精度（换算过程）：
 *     RMT 时钟源 = APB 总线 = 80 MHz
 *     1 tick = 1 / 80,000,000 = 12.5 ns
 *
 *     T0H = 0.40us / 12.5ns = 400/12.5 = 32 ticks  ← 高电平
 *     T0L = 0.85us / 12.5ns = 850/12.5 = 68 ticks  ← 低电平
 *     T1H = 0.80us / 12.5ns = 800/12.5 = 64 ticks
 *     T1L = 0.45us / 12.5ns = 450/12.5 = 36 ticks
 *
 *     验证: T0H+T0L = T1H+T1L = 100 ticks = 1.25 us ✓
 *     一个 bit 耗时 100 ticks，24 bits = 2400 ticks = 30 us（单颗 LED）
 *
 * ============================================================================
 * 二、架构设计
 * ============================================================================
 *
 *   设计原则：
 *     1. 零动态内存分配 — 不用 malloc，杜绝内存泄漏和碎片
 *     2. 资源缓存复用    — 通道和编码器只创建一次，后续调用直接使用
 *     3. 多路独立控制    — 支持在不同 GPIO 上各自管理一条灯带
 *
 *   核心数据结构 — 静态槽位数组：
 *
 *     ┌─────────────────────────────────────────────────────────┐
 *     │                  s_channels[4]                          │
 *     ├──────┬──────────────┬──────────────┬────────────────────┤
 *     │ 槽位 │ gpio_num     │ tx_chan      │ encoder            │
 *     ├──────┼──────────────┼──────────────┼────────────────────┤
 *     │  [0] │ 48 (GPIO48)  │ RMT句柄指针   │ 字节编码器句柄      │
 *     │  [1] │ -1 (空闲)     │ NULL         │ NULL               │
 *     │  [2] │ -1 (空闲)     │ NULL         │ NULL               │
 *     │  [3] │ -1 (空闲)     │ NULL         │ NULL               │
 *     └──────┴──────────────┴──────────────┴────────────────────┘
 *
 *   查找策略：
 *     find_channel(48)  → 遍历数组，匹配 gpio_num == 48  → 返回索引
 *     find_free_slot()  → 遍历数组，匹配 gpio_num == -1  → 返回第一个空闲位
 *
 *   为什么用静态数组（最多 4 路）而不是链表？
 *     — ESP32-S3 只有 8 个 RMT TX 通道，4 路已经够多
 *     — 数组查找 O(4) = 常数时间，绝对可预测
 *     — 无指针追逐，cache 友好
 *     — 不需要链表管理代码，不易出 bug
 *
 * ============================================================================
 * 三、数据流 — 从 API 调用到 GPIO 波形
 * ============================================================================
 *
 *   ws2812_set_rgb(48, 0, 255, 0)    用户调用：GPIO48, G=0, R=255, B=0
 *          │
 *          ▼
 *   ws2812_set_rgb()                  构造 GRB 字节数组
 *     uint8_t grb[3] = {0, 255, 0};   {G, R, B}
 *          │
 *          ▼
 *   ws2812_send(gpio=48, data=grb, num_leds=1)
 *          │
 *          ▼
 *   find_channel(48) → idx=0         查找该 GPIO 对应的 RMT 资源
 *          │
 *          ▼
 *   rmt_transmit(tx_chan, encoder,    异步启动硬件发送
 *                grb, 3, &tx_cfg)     RMT 硬件从内存读取字节
 *          │
 *          ▼
 *   bytes_encoder 工作流程：
 *     字节 0x00 → bit0 bit0 bit0 bit0 bit0 bit0 bit0 bit0
 *     字节 0xFF → bit1 bit1 bit1 bit1 bit1 bit1 bit1 bit1
 *     字节 0x00 → bit0 bit0 bit0 bit0 bit0 bit0 bit0 bit0
 *          │          每个 bit 被翻译为 2 个 RMT 符号（高+低）
 *          ▼
 *   GPIO 引脚输出波形：
 *     ┌─┐ ┌─┐ ┌─┐     ┌──┐┌──┐┌──┐     ┌─┐ ┌─┐ ┌─┐
 *     │ │ │ │ │ │ ... │  ││  ││  │ ... │ │ │ │ │ │ ... ─── RESET (eot_level=0)
 *     └─┘ └─┘ └─┘     └──┘└──┘└──┘     └─┘ └─┘ └─┘
 *          G=0              R=255            B=0
 *          │
 *          ▼
 *   WS2812B 芯片解读：
 *     绿色 PWM = 0   → 绿灯灭
 *     红色 PWM = 255 → 红灯最亮
 *     蓝色 PWM = 0   → 蓝灯灭
 *          │
 *          ▼
 *     LED 显示纯红色 ✓
 *
 *   ═══════════════════════════════════════════════════════════
 *   发送时序关键点：
 *
 *     1. rmt_transmit() 是非阻塞的 — 数据拷贝到 RMT 缓冲区后立即返回
 *     2. rmt_tx_wait_all_done() 必须调用 — 等待硬件发送完成
 *     3. eot_level = 0 — 发送完毕后 GPIO 保持低电平
 *        → WS2812B 将其识别为 RESET 信号（>50us → 锁存新颜色）
 *     4. 如果两次发送之间不等待（漏掉 wait_all_done），
 *        新数据会在旧数据发送完成前覆盖缓冲区 → LED 收到乱码 → 闪烁/颜色错乱
 *
 * ============================================================================
 * 四、关键 Bug 复盘 — static 变量零初始化陷阱
 * ============================================================================
 *
 *   日期：2026-05  |  严重程度：致命（P0）  |  现象：LED 颜色异常（常量蓝灯）
 *
 *   【问题代码】（初始版本）
 *
 *     static ws2812_channel_t s_channels[WS2812_MAX_CHANNELS];
 *     //                                          ^^^ 没有初始化器！
 *
 *   【内存实际状态】
 *
 *     编译器将 s_channels 放入 .bss 段，C 运行时将其全部清零：
 *
 *        s_channels[0].gpio_num = 0   ← 0 是 GPIO 0（真实引脚！）
 *        s_channels[1].gpio_num = 0
 *        s_channels[2].gpio_num = 0
 *        s_channels[3].gpio_num = 0
 *
 *   【错误的连锁反应】
 *
 *     1. find_free_slot() 遍历数组，检查 gpio_num == -1
 *        → 所有值都是 0（不是 -1）→ 返回 -1（无空闲槽位）
 *
 *     2. ws2812_init() 收到 -1 → 认为通道数已满 → 返回 ESP_ERR_NO_MEM
 *
 *     3. main.c 调用 ESP_ERROR_CHECK(ws2812_init(48))
 *        → ESP_ERR_NO_MEM != ESP_OK → abort() → 系统复位
 *
 *     4. 复位后 LED 数据线处于高阻态 → WS2812B 可能解析到随机数据 → 蓝灯常量
 *
 *   【修复方案】
 *
 *     static ws2812_channel_t s_channels[WS2812_MAX_CHANNELS] = {
 *         {.gpio_num = -1},   // 显式初始化每个槽位为"空闲"
 *         {.gpio_num = -1},
 *         {.gpio_num = -1},
 *         {.gpio_num = -1},
 *     };
 *
 *     指定初始化器（designated initializer）只设置 gpio_num，
 *     其余字段（tx_chan, encoder）被自动初始化为 NULL。
 *
 *   【教训】
 *
 *     1. C 语言的零初始化不等于 -1 初始化！
 *        {0} 适合指针（NULL = 0）但不适合哨兵值为 -1 的整数字段。
 *
 *     2. 如果哨兵值不是 0，就必须显式初始化。
 *        替代方案：用单独的 bool in_use 字段，bool 的零值 false 天然是"空闲"。
 *
 *     3. 嵌入式开发中，静态变量的初始化顺序和值是可预测的，
 *        但不能假设"未初始化 = 随机值"——BSS 段是清零的。
 *
 *     4. 如果必须在这个场景使用 -1 哨兵：要么显式初始化，
 *        要么在首次访问时惰性初始化（加一个 s_channels_ready 标志）。
 *
 * ============================================================================
 * 五、错误处理策略
 * ============================================================================
 *
 *   每个公开 API 返回 esp_err_t，调用者用 ESP_ERROR_CHECK 宏检查：
 *
 *     ESP_OK              — 成功
 *     ESP_ERR_INVALID_ARG — 参数无效（gpio_num < 0，data == NULL 等）
 *     ESP_ERR_NOT_FOUND   — GPIO 未初始化（先调 ws2812_init）
 *     ESP_ERR_NO_MEM      — 无空闲通道槽位（超过 WS2812_MAX_CHANNELS）
 *
 *   初始化失败时的回滚（ws2812_init 内部）：
 *
 *     创建 RMT 通道 ✓ → 创建编码器 ✗
 *                        └→ 回滚: 删除 RMT 通道 → 返回错误
 *
 *     创建 RMT 通道 ✓ → 创建编码器 ✓ → 启用通道 ✗
 *                                       └→ 回滚: 删除编码器 → 删除通道 → 返回错误
 *
 *   确保不会泄漏硬件资源（RMT 通道/编码器）。
 *
 * ============================================================================
 * 六、可调参数
 * ============================================================================
 *
 *   WS2812_MAX_CHANNELS (默认 4)
 *     最大可同时控制的独立 GPIO 数。每个通道消耗一个 RMT TX 通道。
 *     ESP32-S3 共有 8 个 TX 通道。如果你只需要 1-2 路，改小可省 RAM。
 *
 *   RMT_RESOLUTION_HZ (80,000,000)
 *     RMT tick 频率。降低可减少功耗，但时序精度会下降。
 *     例如 40MHz → 1 tick = 25ns → T0H 需重新计算 = 0.40us/25ns = 16 ticks。
 *
 *   T0H/T0L/T1H/T1L (32/68/64/36)
 *     如果 LED 颜色异常（偏红/偏绿/闪烁），微调这几项。
 *     WS2812B 数据手册允许的容差: T0H=0.35~0.50us, T1H=0.70~1.00us。
 *     不同厂家（Worldsemi vs 国产兼容品）的容差范围有差异。
 *
 *   RMT_MEM_BLOCK_SYMBOLS (64)
 *     每个符号块 = 64 个 RMT 符号对 = 32 bits = 4 字节。
 *     如果你要一条 DMX 发送大量 LED（数百颗），可能需要在发送时手动分块。
 *
 * ============================================================================
 */

#include "ws2812.h"

#include <string.h>                // memset()

#include "driver/rmt_tx.h"         // ESP-IDF RMT 发送驱动
#include "esp_check.h"             // ESP_RETURN_ON_ERROR 等检查宏
#include "esp_log.h"               // 日志输出
#include "freertos/FreeRTOS.h"     // FreeRTOS 基础类型

/* ── 日志标签 ────────────────────────────────────────────────────────────── */

static const char *TAG = "ws2812";

/* ── 可配置常量 ──────────────────────────────────────────────────────────── */

/*
 * 最多支持的独立 GPIO 通道数。
 *
 * 每个通道消耗一个 RMT 通道（ESP32-S3 共有 8 个 TX 通道）。
 * 如果只需要控制一条灯带，设为 1 或 2 即可节省内存。
 * 增大此值不会显著影响性能，每个通道只占约 40 字节。
 */
#define WS2812_MAX_CHANNELS  4

/*
 * RMT 分辨率（Hz）= tick 频率
 *
 * 80MHz → 1 tick = 12.5ns。
 * 这个频率等于 ESP32-S3 的 APB 总线时钟，RMT 直接使用无需分频。
 */
#define RMT_RESOLUTION_HZ    80000000

/*
 * WS2812B 时序参数（以 RMT tick 为单位 @ 80MHz）
 *
 * 换算：1 us = 1000 ns，1 tick = 12.5 ns
 *   T0H = 0.40us / 12.5ns = 400 / 12.5 = 32 ticks
 *   T0L = 0.85us / 12.5ns = 850 / 12.5 = 68 ticks
 *   T1H = 0.80us / 12.5ns = 800 / 12.5 = 64 ticks
 *   T1L = 0.45us / 12.5ns = 450 / 12.5 = 36 ticks
 *
 * 验证: T0H+T0L = T1H+T1L = 100 ticks = 1.25 us ✓
 *
 * 如果 LED 颜色异常（偏色、闪烁），首先检查这几个值
 * 是否在 WS2812B 数据手册允许的范围内。
 */
#define T0H  32    // bit 0 高电平: 0.40 us
#define T0L  68    // bit 0 低电平: 0.85 us
#define T1H  64    // bit 1 高电平: 0.80 us
#define T1L  36    // bit 1 低电平: 0.45 us

/*
 * RMT 内存块大小（以 RMT 符号对为单位）
 *
 * 每个 bit 需要 2 个 RMT 符号（一个高+一个低）。
 * 一个 MBS=64 的内存块可以存储 64 个符号对 = 32 个 bit = 4 字节。
 *
 * 对于 3 字节（单颗 LED），需要至少 24 个符号对，64 完全够用。
 * 对于 N 颗 LED 的灯带，需要 N*24 个符号对。
 * 如果 N 很大（>128），可以增大此值，或依赖 RMT 的 DMA 功能分块发送。
 */
#define RMT_MEM_BLOCK_SYMBOLS  64

/*
 * 发送队列深度
 *
 * 设为 1 表示同一时刻最多有一个发送请求排队。
 * 对于单任务场景足够，如果需要多任务并发发送，可以增大。
 */
#define RMT_TRANS_QUEUE_DEPTH  1

/* ── 通道上下文结构 ──────────────────────────────────────────────────────── */

typedef struct {
    int gpio_num;                        // GPIO 编号，-1 表示此槽位空闲
    rmt_channel_handle_t tx_chan;        // RMT 发送通道句柄
    rmt_encoder_handle_t encoder;        // 字节编码器句柄
} ws2812_channel_t;

/*
 * 通道数组 — 缓存所有已初始化的 GPIO
 *
 * 约定:
 *   - s_channels[i].gpio_num == -1  → 槽位空闲
 *   - s_channels[i].gpio_num >= 0   → 槽位被占用，即该 GPIO 已初始化
 *
 * 查找某个 GPIO 对应的通道时，遍历此数组匹配 gpio_num。
 * 插入新通道时，找第一个 gpio_num == -1 的空槽位。
 *
 * 注意: 用静态数组而非链表，牺牲了一点灵活性（最多 4 路），
 *       换取了零动态内存分配和 O(1) 确定性的内存占用。
 *       这对嵌入式系统非常重要——没有 malloc 就没有内存泄漏和碎片。
 */
/*
 * 注意：必须显式初始化 gpio_num = -1！
 * C 语言中 static 变量默认零初始化，gpio_num=0 会被误判为"GPIO 0 已占用"，
 * 导致 find_free_slot() 找不到空闲槽位，ws2812_init() 返回 ESP_ERR_NO_MEM。
 */
static ws2812_channel_t s_channels[WS2812_MAX_CHANNELS] = {
    {.gpio_num = -1},
    {.gpio_num = -1},
    {.gpio_num = -1},
    {.gpio_num = -1},
};

/* ── 内部辅助函数 ────────────────────────────────────────────────────────── */

/*
 * 在 s_channels 中查找 gpio_num 对应的通道索引
 *
 * 返回:
 *   >= 0  — 通道在数组中的索引
 *   -1    — 未找到（该 GPIO 未初始化）
 *
 * 时间复杂度: O(WS2812_MAX_CHANNELS)，对于 MAX=4 是常数时间。
 */
static int find_channel(int gpio_num)
{
    for (int i = 0; i < WS2812_MAX_CHANNELS; i++) {
        if (s_channels[i].gpio_num == gpio_num) {
            return i;
        }
    }
    return -1;
}

/*
 * 在 s_channels 中找一个空闲槽位
 *
 * 返回:
 *   >= 0  — 空闲槽位的索引
 *   -1    — 无空闲槽位（所有通道都在用）
 */
static int find_free_slot(void)
{
    for (int i = 0; i < WS2812_MAX_CHANNELS; i++) {
        if (s_channels[i].gpio_num == -1) {
            return i;
        }
    }
    return -1;
}

/*
 * 创建一个 RMT 字节编码器
 *
 * 编码器负责把原始字节（如 0x80）翻译成 RMT 符号序列。
 * 这是 ESP-IDF RMT 驱动最方便的功能：你不需要手动为每个 bit
 * 计算符号，编码器按你定义的 bit0/bit1 规则自动生成。
 *
 * 参数:
 *   ret_encoder — 输出，创建的编码器句柄
 * 返回:
 *   ESP_OK 或错误码
 */
static esp_err_t create_encoder(rmt_encoder_handle_t *ret_encoder)
{
    /*
     * rmt_bytes_encoder_config_t — 字节编码器的配置
     *
     * 核心就是这个结构体：定义 bit0 和 bit1 各自对应的 RMT 波形。
     */
    rmt_bytes_encoder_config_t enc_cfg = {
        /*
         * bit0 — "0" 码的编码
         *
         *   符号0: 高电平持续 T0H ticks
         *   符号1: 低电平持续 T0L ticks
         *
         * level0 = 1 (高电平), level1 = 0 (低电平)
         * duration0/duration1 是持续时间，单位是 RMT tick
         */
        .bit0 = {
            .duration0 = T0H, .level0 = 1,   // ─┐‾‾‾┌─ 高电平部分
            .duration1 = T0L, .level1 = 0,   //   └───┘  低电平部分
        },

        /*
         * bit1 — "1" 码的编码
         */
        .bit1 = {
            .duration0 = T1H, .level0 = 1,   // ─┐‾‾‾‾‾┌─ 高电平部分
            .duration1 = T1L, .level1 = 0,   //   └─────┘  低电平部分
        },

        /*
         * .flags.msb_first = 1
         *
         * 每个字节的 8 个 bit 从最高位（MSB, bit7）开始发送。
         * WS2812B 协议强制要求 MSB first。
         *
         * 例如字节 0x80 (二进制 1000_0000):
         *   发送顺序: 1 → 0 → 0 → 0 → 0 → 0 → 0 → 0
         *
         * 如果误设为 LSB first，LED 会收到完全错误的颜色数据。
         */
        .flags.msb_first = 1,
    };

    return rmt_new_bytes_encoder(&enc_cfg, ret_encoder);
}

/* ── 公共 API ────────────────────────────────────────────────────────────── */

/*
 * ws2812_init — 初始化 WS2812B 驱动
 *
 * 完整的初始化流程:
 *
 *   ┌──────────────────┐
 *   │ 1. 参数校验       │ → gpio_num 必须 >= 0
 *   ├──────────────────┤
 *   │ 2. 查重           │ → 如果已初始化，直接返回 ESP_OK（幂等）
 *   ├──────────────────┤
 *   │ 3. 找空槽位       │ → 遍历 s_channels[]
 *   ├──────────────────┤
 *   │ 4. 创建 RMT 通道  │ → rmt_new_tx_channel()
 *   ├──────────────────┤
 *   │ 5. 创建编码器     │ → create_encoder()
 *   ├──────────────────┤
 *   │ 6. 启用 RMT 通道  │ → rmt_enable()
 *   ├──────────────────┤
 *   │ 7. 记录到数组     │ → s_channels[slot] = {gpio, chan, encoder}
 *   └──────────────────┘
 *
 * 失败时的回滚: 如果第 4/5/6 步失败，会回滚已创建的资源。
 *   ESP-IDF 的 ESP_RETURN_ON_ERROR 宏会帮我们处理——出错就直接返回，
 *   但这里我们需要手动清理，所以用 ESP_GOTO_ON_ERROR 跳转到清理代码。
 */
esp_err_t ws2812_init(int gpio_num)
{
    ESP_LOGI(TAG, "Initializing WS2812 on GPIO %d", gpio_num);

    /* 1. 参数校验 */
    if (gpio_num < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 2. 查重 — 同一个 GPIO 只初始化一次（幂等） */
    if (find_channel(gpio_num) >= 0) {
        ESP_LOGW(TAG, "GPIO %d already initialized, skipping", gpio_num);
        return ESP_OK;
    }

    /* 3. 找空槽位 */
    int slot = find_free_slot();
    if (slot < 0) {
        ESP_LOGE(TAG, "No free channel slot (max %d)", WS2812_MAX_CHANNELS);
        return ESP_ERR_NO_MEM;
    }

    /*
     * 中间资源变量，初始化为 NULL
     *
     * 如果初始化过程中某一步失败，我们需要清理已完成的部分。
     * 用 NULL 判断"哪些资源已经被创建了"。
     */
    rmt_channel_handle_t tx_chan = NULL;
    rmt_encoder_handle_t encoder = NULL;
    esp_err_t ret;

    /*
     * 4. 创建 RMT 发送通道
     *
     * rmt_tx_channel_config_t 各字段含义:
     *   .gpio_num     — 信号输出引脚
     *   .clk_src      — 时钟源 (RMT_CLK_SRC_DEFAULT = 80MHz APB 时钟)
     *   .resolution_hz — tick 频率 (80MHz = 12.5ns 分辨率)
     *   .mem_block_symbols — 内存块大小 (64 个符号对)
     *   .trans_queue_depth — 发送队列深度 (1)
     */
    rmt_tx_channel_config_t tx_chan_cfg = {
        .gpio_num           = gpio_num,
        .clk_src            = RMT_CLK_SRC_DEFAULT,
        .resolution_hz      = RMT_RESOLUTION_HZ,
        .mem_block_symbols  = RMT_MEM_BLOCK_SYMBOLS,
        .trans_queue_depth  = RMT_TRANS_QUEUE_DEPTH,
    };

    ret = rmt_new_tx_channel(&tx_chan_cfg, &tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
        return ret;  // tx_chan 创建失败，无需清理
    }

    /* 5. 创建字节编码器 */
    ret = create_encoder(&encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create bytes encoder: %s", esp_err_to_name(ret));
        // 回滚: 删除已创建的 RMT 通道
        rmt_del_channel(tx_chan);
        return ret;
    }

    /* 6. 启用 RMT 通道（打开时钟和电源） */
    ret = rmt_enable(tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        // 回滚: 按创建的反序清理
        rmt_del_encoder(encoder);
        rmt_del_channel(tx_chan);
        return ret;
    }

    /* 7. 记录到通道数组 */
    s_channels[slot].gpio_num = gpio_num;
    s_channels[slot].tx_chan  = tx_chan;
    s_channels[slot].encoder  = encoder;

    ESP_LOGI(TAG, "WS2812 on GPIO %d initialized (slot %d)", gpio_num, slot);
    return ESP_OK;
}

/*
 * ws2812_set_rgb — 设置单颗 LED 颜色
 *
 * 这是最常用的函数。内部流程:
 *   1. 在 s_channels 中查找 gpio_num
 *   2. 构造 3 字节 GRB 数据
 *   3. 通过 RMT 发送
 *   4. 等待发送完成
 *
 * 性能:
 *   - 3 字节数据 = 24 bit = 48 个 RMT 符号
 *   - 每个符号 100 ticks @ 80MHz = 1.25us
 *   - 总发送时间约 30us（加上函数调用开销约 50us）
 *   - 对于呼吸灯（20ms/帧）来说完全不是瓶颈
 */
esp_err_t ws2812_set_rgb(int gpio_num, uint8_t g, uint8_t r, uint8_t b)
{
    /* 1. 查找通道 */
    int idx = find_channel(gpio_num);
    if (idx < 0) {
        ESP_LOGE(TAG, "GPIO %d not initialized, call ws2812_init() first", gpio_num);
        return ESP_ERR_NOT_FOUND;
    }

    /*
     * 2. 构造 GRB 数据帧
     *
     * WS2812B 的颜色格式: G → R → B（绿→红→蓝），高位先发
     *
     * 如果传入了 g=0, r=255, b=0:
     *   发送字节序列: 0x00, 0xFF, 0x00
     *   发送 bit 序列:
     *     G=0x00: 00000000 00000000
     *     R=0xFF: 11111111 11111111
     *     B=0x00: 00000000 00000000
     *
     *   绿色通道全 0 → 绿灯不亮
     *   红色通道全 1 → 红灯最亮
     *   蓝色通道全 0 → 蓝灯不亮
     *   → LED 显示纯红色 ✓
     */
    uint8_t grb[3] = {g, r, b};

    /* 3-4. 发送并等待完成（复用 ws2812_send 的逻辑） */
    return ws2812_send(gpio_num, grb, 1);
}

/*
 * ws2812_send — 发送数据到多颗级联 LED
 *
 * 这是 ws2812_set_rgb 的通用版本，ws2812_set_rgb 实际上是
 * ws2812_send(gpio, &grb, 1) 的包装。
 *
 * 如果要控制一条 8 颗 LED 的灯带:
 *
 *   uint8_t data[24];  // 8 颗 LED × 3 字节
 *   data[0]=255; data[1]=0; data[2]=0;    // LED0 = 绿色
 *   data[3]=0;   data[4]=255; data[5]=0;  // LED1 = 红色
 *   // ... 继续设置 LED2-LED7
 *   ws2812_send(48, data, 8);
 *
 * WS2812B 的级联机制:
 *   - LED0 从 data[0..2] 取自己的颜色，然后把 data[3..] 转发给 LED1
 *   - LED1 从 data[3..5] 取自己的颜色，然后把 data[6..] 转发给 LED2
 *   - ...
 *   - 最后一颗 LED 收完自己的颜色后，不再转发
 *
 *   发送完成后，数据线保持低电平 >50us（RESET 信号），
 *   所有 LED 同时锁存新颜色。
 */
esp_err_t ws2812_send(int gpio_num, const uint8_t *grb_data, size_t num_leds)
{
    /* 参数校验 */
    if (grb_data == NULL || num_leds == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 查找通道 */
    int idx = find_channel(gpio_num);
    if (idx < 0) {
        ESP_LOGE(TAG, "GPIO %d not initialized, call ws2812_init() first", gpio_num);
        return ESP_ERR_NOT_FOUND;
    }

    ws2812_channel_t *ch = &s_channels[idx];

    /*
     * 发送配置
     *
     * .loop_count = 0
     *   不循环发送。设为 -1 会让 RMT 无限循环发送同一段数据，
     *   这对 WS2812 没有意义（每次都需要更新颜色）。
     *
     * .flags.eot_level = 0
     *   发送完成后将 GPIO 拉低。这对于 WS2812B 至关重要！
     *   发送结束后数据线保持低电平超过 50us，WS2812B 才会将其
     *   识别为 RESET 信号并锁存新颜色。
     *   如果把这里设为 1（高电平），LED 不会更新显示。
     */
    rmt_transmit_config_t tx_cfg = {
        .loop_count     = 0,
        .flags.eot_level = 0,
    };

    size_t data_len = num_leds * 3;

    /*
     * rmt_transmit() — 启动异步发送
     *
     * 这个函数是非阻塞的——它把数据拷贝到 RMT 硬件缓冲区后立即返回。
     * 实际的信号产生和 GPIO 操控完全由 RMT 硬件完成。
     *
     * 参数:
     *   ch->tx_chan — RMT 发送通道
     *   ch->encoder — 字节编码器（把 grb_data 中的字节转为 RMT 符号）
     *   grb_data    — 颜色数据
     *   data_len    — 数据长度（字节数 = LED 数 × 3）
     *   &tx_cfg     — 发送选项
     */
    esp_err_t ret = rmt_transmit(ch->tx_chan, ch->encoder,
                                  grb_data, data_len, &tx_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * rmt_tx_wait_all_done() — 阻塞等待硬件发送完成
     *
     * portMAX_DELAY 表示无限等待。对于少量 LED（<100 颗），
     * 发送时间在微秒到毫秒级，几乎不会阻塞。
     *
     * 必须调用这个函数！如果不等待，下一次 rmt_transmit 可能会
     * 在上次发送未完成时覆盖缓冲区，导致 LED 收到乱码。
     */
    ret = rmt_tx_wait_all_done(ch->tx_chan, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT wait done failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/*
 * ws2812_deinit — 释放资源
 *
 * 按创建的反序释放:
 *   1. 禁用 RMT 通道
 *   2. 删除编码器
 *   3. 删除 RMT 通道
 *   4. 标记槽位为空
 *
 * 即使某一步释放失败，也会继续释放剩余资源。
 */
void ws2812_deinit(int gpio_num)
{
    int idx = find_channel(gpio_num);
    if (idx < 0) {
        ESP_LOGW(TAG, "GPIO %d not initialized, nothing to deinit", gpio_num);
        return;
    }

    ws2812_channel_t *ch = &s_channels[idx];

    // 禁用通道（如果失败也继续清理，因为通道对象仍需要释放）
    rmt_disable(ch->tx_chan);
    // 删除编码器和通道
    rmt_del_encoder(ch->encoder);
    rmt_del_channel(ch->tx_chan);

    // 清空槽位
    memset(ch, 0, sizeof(*ch));
    ch->gpio_num = -1;

    ESP_LOGI(TAG, "WS2812 on GPIO %d deinitialized", gpio_num);
}
