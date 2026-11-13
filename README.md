# 双轮差速底盘

基于 STM32F103C8T6、STM32 HAL 和 CMake。遵循 AGENTS.md 的中文注释、下划线命名及 clang-format 约定。

## 原实现与本次整理

原 chassis_move 将 v、omega 换算为左右轮速，chassis_set_wheel 等比限幅，
chassis_update 独立限制每轮加速度，再用 mmps_to_duty 做线性映射和死区补偿。
编码器通过 16 位计数器差分获取增量，odom_update 换算轮速并低通滤波。
主循环原先未启动电机/编码器，也未调用这些更新函数。

原底盘满量程为 1000，底层却把输入截断到 100，并直接写 ARR=99 的定时器。
原死区值 180 已超过底层限幅，因此非零运动容易直接成为满占空比。
现在统一使用千分比，BSP 根据实际 ARR 换算。例如 180 表示 18%，ARR=99 时 CCR=18。
不改变 CubeMX 配置及 PWM 频率。

## 调用关系

main 每约 10ms 使用实际经过时间调用 chassis_update：
1. motor_speed_update 是编码器的唯一周期采样入口，维护累计计数和速度缓存。
2. chassis 对目标轮速施加加速度限制。
3. app_motor_update 将目标轮速、缓存实测轮速及 dt_s 传给单轮控制入口。
4. app_motor_control_step 当前做开环映射，将来在此接左右轮独立速度 PID。
5. bsp_motor_set_pwm 处理接线方向、限幅和定时器 CCR 换算。

累计计数只在 motor_speed 保存，反馈通过 motor_speed_get_feedback 获取副本。
初始化和 motor_speed_reset 同时重建采样基准、清零累计量及滤波状态。
运行中手动复位测速时应停车，并重新建立主循环采样时间基准。
所有应用接口由主循环调用；若以后跨中断使用，需要另行处理数据同步。

## 引脚与单位

引脚配置.md 规定 TIM4 四路 PWM 分别驱动两台电机、TIM2/TIM3 用作编码器。
下表的具体 GPIO 和左右轮对应关系来自当前工程的 CubeMX 配置及 BSP 绑定。

| 功能 | 定时器 | GPIO |
| --- | --- | --- |
| 左轮双输入驱动 | TIM4 CH1 / CH2 | PB6 / PB7 |
| 右轮双输入驱动 | TIM4 CH3 / CH4 | PB8 / PB9 |
| 左轮编码器 | TIM3 CH1 / CH2，部分重映射 | PB4 / PB5 |
| 右轮编码器 | TIM2 CH1 / CH2 | PA0 / PA1 |

电机方向沿用左 +1、右 -1；编码器方向沿用两侧 +1，分别在 BSP 绑定表配置。
车轮向前滚动时反馈应为正，驱动方向与编码器方向必须独立实测。
文档描述为磁编码器，软件仅依赖其 A/B 相正交输出，不依赖光电或磁检测原理。

- 轮速、车体线速度：mm/s，前进为正。
- 角速度：rad/s，俯视逆时针为正。
- 时间：外层 dt_ms 为毫秒，控制器入口 dt_s 为秒。
- 输出：pwm_permille，范围 -1000 到 +1000。
- 轮距、半径：mm。参数集中在 app/Inc/chassis_config.h。

轮距150mm、半径50mm、减速比35、编码器11线、最大轮速600mm/s、
启动补偿180千分比、加速度6000mm/s²均沿用原工程，不能作为本次实测结果。
11线应核实为电机轴单通道每转脉冲数；按此解释每轮一圈1540个计数。
10ms窗口的单计数速度分辨率约20.4mm/s，低速时需注意测速量化和滤波延迟。

## 常用接口与命名迁移

| 原名称 | 当前名称/处理 |
| --- | --- |
| chassis_move | chassis_set_velocity |
| chassis_set_wheel | chassis_set_wheel_speeds |
| chassis_brake | chassis_request_stop（按斜坡减速，不代表主动刹车） |
| chassis_stop_now | chassis_stop（清零指令和输出，不经过斜坡） |
| left/right_motor_set_speed | bsp_motor_set_pwm，明确单位为千分比 |
| odom_update | motor_speed_update，由底盘统一调度 |
| encoder_get_delta | bsp_encoder_read_delta，仅测速模块调用 |
| wheel_total_mm | motor_speed_get_distance_mm |
| 全局 wheel、enc.total | 私有 motor_feedback 及只读副本查询，累计量仅一份 |
| s_tgt_l/r、s_out_l/r | target_wheel_speed_mm_s、ramped_wheel_speed_mm_s |
| pulse_left/right、app_motor_test/drive | 移除未使用变量和不一致的空接口 |

初始化返回 bool；失败时保持零输出，由 main 进入 Error_Handler。
启动后默认目标为零，示例命令不会自动执行。完成硬件核验后可在应用层调用：

~~~c
chassis_set_velocity(200.0f, 0.0f); /* 前进目标200mm/s。 */
chassis_set_velocity(0.0f, 1.0f);   /* 逆时针原地旋转目标1rad/s。 */
chassis_request_stop();             /* 后续周期按斜坡减速。 */
chassis_stop();                     /* 清除目标和斜坡，PWM请求归零。 */
~~~

这些是开环目标，实际速度取决于电源、负载和标定。两个输入置低的实际停机方式
取决于驱动芯片；此工程没有据此声称主动刹车。PWM预装载在下个更新事件生效。

## PID 扩展

在 app_motor_control_step(motor_id, target_speed_mm_s, measured_speed_mm_s, dt_s)
内添加速度环；motor_id 区分左右轮，各自维护积分和历史误差。
输出仍为千分比并限幅，app_motor_stop 和初始化时应复位 PID 状态。
当前未实现 PID、未填入未经标定的 PID 系数，编码器反馈已接到入口。

## 构建与验证

确保 arm-none-eabi-gcc、CMake 和 Ninja 在 PATH 中：

~~~sh
cmake --preset Debug
cmake --build --preset Debug
~~~

独立主机回归测试使用原应用/BSP源文件和最小 HAL 寄存器替身：

~~~sh
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
~~~

测试覆盖千分比转换、右轮极性、正反计数回绕、重复读反馈、零时间步、清零、
差速转向、比例限幅、加减速斜坡、停止、非法输入和初始化失败。
主机替身不模拟 PWM 电气时序、实际驱动板和电机，实际接线、方向、速度参数仍需上板验证。
