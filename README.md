# ESP32-RMT-ws2812-
封装好的驱动，放在components中即可，已包含CMakeLists.txt
# ws2812 — ESP32 WS2812B RMT 驱动库

基于 ESP-IDF RMT 外设的 WS2812B 可寻址 RGB LED 驱动，单文件、零依赖（仅需 `esp_driver_rmt`），零动态内存分配。

## 特性

- **RMT 硬件驱动** — 80MHz / 12.5ns 精度，不受 CPU 中断影响
- **API 极简** — 4 个函数覆盖所有场景
- **多路并行** — 最多 4 个 GPIO 各自独立控制
- **零 malloc** — 静态数组管理通道，无内存泄漏风险
- **幂等初始化** — 同一 GPIO 多次 `init` 安全返回
- **失败回滚** — 初始化中途失败自动清理已创建资源

## 安装

将 `components/ws2812/` 目录复制到你的 ESP-IDF 项目根目录。确保根 `CMakeLists.txt` 中包含 `components/` 路径：

```cmake
set(EXTRA_COMPONENT_DIRS ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/components)
```

## API

```c
#include "ws2812.h"

esp_err_t ws2812_init(int gpio_num);                                     // 初始化
esp_err_t ws2812_set_rgb(int gpio_num, uint8_t g, uint8_t r, uint8_t b);  // 单颗 LED
esp_err_t ws2812_send(int gpio_num, const uint8_t *grb_data, size_t n);   // 灯带
void      ws2812_deinit(int gpio_num);                                   // 释放
```

## 快速上手

```c
// 初始化（仅一次）
ESP_ERROR_CHECK(ws2812_init(48));

// 设置颜色
ws2812_set_rgb(48, 0, 255, 0);      // 红色  (G=0, R=255, B=0)
ws2812_set_rgb(48, 0, 0, 255);      // 蓝色
ws2812_set_rgb(48, 255, 255, 255);  // 白色

// 灯带
uint8_t buf[6] = {0,255,0, 0,0,255};  // LED0=红, LED1=蓝
ws2812_send(48, buf, 2);
```

> **⚠️ 颜色顺序是 GRB（绿-红-蓝），不是 RGB！** 红色 = `(0, 255, 0)`，绿色 = `(255, 0, 0)`。

## 示例：呼吸灯

```c
ws2812_init(48);
float phase = 0.0f;
while (1) {
    uint8_t r = (uint8_t)(sinf(phase) * sinf(phase) * 255.0f);
    ws2812_set_rgb(48, 0, r, 0);
    phase += 0.03f;
    if (phase > 6.283f) phase -= 6.283f;
    vTaskDelay(pdMS_TO_TICKS(20));
}
```

## 示例：双路独立控制

```c
ws2812_init(48);   // 灯带 A
ws2812_init(12);   // 灯带 B

ws2812_set_rgb(48, 255, 0, 0);    // A = 绿色
ws2812_set_rgb(12, 0, 0, 255);    // B = 蓝色
```

## 原理

WS2812B 使用单线 NRZ 协议，通过高电平脉宽区分 0/1 码。ESP32 的 RMT 外设以 80MHz 分辨率产生精确波形：

```
T0H=0.40µs  T0L=0.85µs  →  bit "0"
T1H=0.80µs  T1L=0.45µs  →  bit "1"
```

内部用静态数组缓存每个 GPIO 的 RMT 通道和编码器句柄，`init` 时创建，`set_rgb` 时直接复用，不会重复创建。

## 可调参数

在 `ws2812.c` 顶部：

| 宏 | 默认 | 说明 |
|---|---|---|
| `WS2812_MAX_CHANNELS` | 4 | 最大独立 GPIO 路数 |
| `T0H / T0L` | 32 / 68 | "0" 码脉宽（RMT tick） |
| `T1H / T1L` | 64 / 36 | "1" 码脉宽（RMT tick） |
| `RMT_RESOLUTION_HZ` | 80,000,000 | RMT 时钟频率 |

LED 颜色异常时可微调 `T0H/T1H`。

## 依赖 & 兼容性

- **ESP-IDF v5+**：将 `CMakeLists.txt` 中 `REQUIRES "esp_driver_rmt"` 改为 `REQUIRES "driver"`
- **ESP-IDF v6+**：使用 `REQUIRES "esp_driver_rmt"`

## 许可证

MIT
