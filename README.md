# 基于 CAN 的 AI 危险事件联动控制（PC 检测 + STM32 声光执行）

## 一、项目简介

PC 端用 YOLOv8n 对离线视频做人员检测，判断人员是否进入危险区域 ROI，按**停留帧数**分出安全 / 黄色告警 / 红色告警三态，通过 USB-CAN（PCAN-USB）以 CAN 总线把状态命令发给 STM32F103；STM32 查表驱动 LED 和有源蜂鸣器做出对应声光动作，并回 ACK 应答。PC 侧实现序号配对、超时检测与 3 次重试，形成可靠的命令-应答闭环。

> 本项目是项目1（离线视频人员危险区域入侵检测）的下游延伸：项目1负责"判断有没有人进来"，本项目负责"把判断结果通过 CAN 送到执行节点并驱动声光报警"。

## 二、系统架构

```text
[PC: YOLOv8n 检测 + 状态机 + 可靠发送]
        │ USB
        ▼
   PCAN-USB 适配器 (500 kbps)
        │ CAN_H / CAN_L（两端 120Ω 终端电阻）
        ▼
[STM32F103: CAN 接收中断 → 查表 → TIM2 中断闪鸣 → 回 ACK]
```

- PC 只在**状态切换边沿**发一次命令，不每帧重复发；
- STM32 是状态保持型：收到命令后持续执行，直到收到新命令；
- ACK 在 CAN 接收中断里立即回，闪鸣节奏由独立 TIM2 定时中断驱动，主循环不阻塞。

## 三、硬件与接线

| 项 | 配置 |
|---|---|
| PC 适配器 | PEAK PCAN-USB，PCAN-View / python-can pcan 后端 |
| STM32 | F103，HAL 库，CAN1：PA11=RX / PA12=TX |
| 波特率 | 500 kbps（Prescaler=9, BS1=6TQ, BS2=1TQ, SJW=1TQ，NORMAL 模式） |
| 终端电阻 | 总线两端各 120Ω |
| 执行器件 | 绿/黄/红 LED + 有源蜂鸣器，均为**低电平导通**（点亮=RESET，熄灭=SET） |
| 显示 | 0.96 寸 OLED（I2C），显示当前状态 / 人数 / 命令序号 |

## 四、环境依赖（PC 侧 venv：D:\venvs\embedded-ai）

```powershell
D:\venvs\embedded-ai\Scripts\python.exe -m pip install python-can ultralytics opencv-python numpy
```

- python-can 4.6.1，pcan 后端（需 PEAK 驱动，PCAN-View 已装即带）；
- ultralytics（YOLOv8n）、torch、opencv-python、numpy；
- 本地权重：`D:\embedded-ai\scripts\YOLO_Learning\yolov8n.pt`。

## 五、CAN 协议（第一版实际实现）

### 1. CAN ID

| CAN ID | 方向 | 内容 |
|---|---|---|
| `0x202` | PC → STM32 | 状态命令帧，8 字节 |
| `0x301` | STM32 → PC | ACK 应答帧，8 字节 |
| `0x7F0` | 双向 | 心跳帧，PC 每秒发、STM32 收到即回 |

### 2. 0x202 命令帧（PC → STM32）

| 字节 | 字段 | 取值 |
|---|---|---|
| Byte0 | 命令类型 | 0=NORMAL 安全，1=WARNING 黄警，2=DANGER 红警（3=ESTOP 预留未实现） |
| Byte1 | 区内人数 | 实时检出人数，供参考 |
| Byte2 | 命令序号 | 每条命令 +1，ACK 配对用 |
| Byte3~7 | 预留 | 填 0 |

### 3. 0x301 ACK 帧（STM32 → PC）

| 字节 | 字段 | 取值 |
|---|---|---|
| Byte0 | 命令序号 | 原样抄回 PC 发来的序号 |
| Byte1 | 执行状态 | 0=成功 |
| Byte2 | 错误码 | 0=无错 |
| Byte3~7 | 预留 | 填 0 |

### 3.1 0x7F0 心跳帧（双向）

PC 每秒发一帧，STM32 在 CAN 接收中断里收到后立刻回一帧，PC 靠"连续3秒没收到回包"判定 STM32 掉线。

| 字节 | 字段 | 取值 |
|---|---|---|
| Byte0 | 节点 ID | 1=PC，2=STM32 |
| Byte1 | 节点状态 | 0=运行，1=停止 |
| Byte2~7 | 预留 | 填 0 |

> 过滤器用 32 位列表模式同时放行 0x202 和 0x7F0 两个 ID（Bank0 两个槽）。

### 4. 状态 → 动作映射（STM32 查表）

| Byte0 | 状态 | 灯光 | 蜂鸣器 |
|---|---|---|---|
| 0 | NORMAL | 绿灯常亮 | 停 |
| 1 | WARNING | 黄灯慢闪（800ms） | 慢响（800ms 间隔） |
| 2 | DANGER | 红灯急闪（150ms） | 急促连叫（150ms 间隔） |

### 5. PC 侧分级规则（第一版实际：按停留帧数，不按人数）

- 外层防抖：连续 5 帧有人在内 → 进 ALARM；连续 10 帧无人 → 回 NORMAL；
- ALARM 内按连续在内帧数 `intrusion_frames` 分级：
  - `< 30` 帧 → cmd1 黄警；
  - `≥ 30` 帧 → cmd2 红警。

### 6. 可靠发送（第一版实际参数）

- 发前清空接收缓冲残留旧帧；
- 超时 1.0 秒（`deadline = time.time()+1.0`），0.1 秒短轮询；
- 最多重试 3 次，成功才更新 `last_cmd` 并标记完成；
- 序号配对：ACK Byte0 == 本次 this_seq 才算应答成功。

## 六、运行方式

1. 接线：PCAN-USB 与 STM32 的 CAN_H/CAN_L 对接、共地，确认 PCAN-View 已退出（避免占用通道）；
2. 烧录 STM32 固件（CAN + TIM2 + GPIO + OLED）；
3. 命令行运行 PC 脚本：

```powershell
D:\venvs\embedded-ai\Scripts\python.exe "D:\embedded-ai\scripts\Projects_\基于CAN的AI危险事件联动控制\main.py"
```

4. 画面显示检测框与状态，控制台打印序号匹配 / 第几次重试 / 超时失败；按 q 退出。

## 七、关键参数表

| 参数 | 值 | 位置 |
|---|---|---|
| CAN 波特率 | 500 kbps | STM32 MX_CAN_Init / PC Bus bitrate |
| 进 ALARM 防抖 | 连续 5 帧 | main.py intrusion_frames>=5 |
| 解除防抖 | 连续 10 帧 | main.py leave_frames>=10 |
| 黄/红分级阈值 | intrusion_frames=30 | main.py |
| TIM2 时基 | PSC=719, ARR=999 → 10ms 中断 | tim.c |
| 黄闪/黄鸣 | 80 次 ×10ms = 800ms | STM32 TIM 回调 |
| 红闪/红鸣 | 15 次 ×10ms = 150ms | STM32 TIM 回调 |
| ACK 超时 | 1.0s，短轮询 0.1s，重试 3 次 | main.py |

## 八、实测结果（pedestrians.avi）

- 正常链路：序号全部配对，ACK 均为 0x301、状态0、错误码0；
- 拔地线断线：超时分支正确触发，3 次重试失败后放弃；恢复后下一条命令第 1/2 次重试成功，不错位；
- 心跳掉线：STM32 断电/拔线 3 秒后 PC 进"设备连接异常"故障态卡住提醒；接线恢复后连续收到心跳自动 break 恢复；
- 性能：推理约 13~15 ms/帧，单帧总耗时约 14~19 ms，理论上限约 50~76 FPS（瓶颈在模型推理，CAN 发送耗时可忽略）。

## 九、第一版未实现（后续增强）

- ESTOP(cmd3) 急停锁存；
- 失败命令不消耗序号、序号回绕处理；
- 1↔2 分级内部迟滞防抖（当前用停留帧数，已规避人数抖动）；
- 真实摄像头实时流替代离线视频。
