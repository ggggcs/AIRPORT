> v6单位更新：用户实测2754对应275.4mm，以下历史raw参数说明以本段为准。代码配置使用CLAMP_AXIS1_START_MM/END_MM和CLAMP_AXIS2_START_MM/END_MM，填写毫米（例如275.4f）。运行时Clamp_SetStart/End/Endpoints接口同样传入毫米float。START/END/TOL串口参数现在是毫米，可含小数，旧版raw命令请勿直接照用。JOG角度命令和GRIP/RELEASE保持不变。串口输出distance_mm、start_mm、end_mm、target_mm、tol_mm，一位小数；valid=0时距离为上次有效值，不可作为当前测量。

# v5 位置闭环示教与测试

## 直接在代码中设置起点、终点

修改 `Core/Inc/clamp_control.h` 顶部四个参数，再从 VS Code 编译下载：

```c
#define CLAMP_AXIS1_START_MM 0.0f  /* 轴1松开位置 */
#define CLAMP_AXIS1_END_MM   0.0f  /* 轴1抓紧位置 */
#define CLAMP_AXIS2_START_MM 0.0f  /* 轴2松开位置 */
#define CLAMP_AXIS2_END_MM   0.0f  /* 轴2抓紧位置 */
```

参数为传感器绝对距离raw，不是角度或移动增量。0表示尚未配置；改成实际测得的位置后，上电自动加载，无需示教。每轴两端点间距须大于两倍容差。不会上电自动运动，仍需GRIP/RELEASE命令。

后续网口解析层将收到的值排队到主循环调用：

```c
int ok = Clamp_SetEndpoints(1, received_start, received_end); /* 原子更新一轴两端点 */
/* 或单独修改 */
ok = Clamp_SetStart(1, received_start);
ok = Clamp_SetEnd(1, received_end);
```

axis为1或2，返回1表示接受、0表示拒绝。运动中或故障状态拒绝修改；数值范围1～1000000 raw；两参数同时设置时也拒绝相同端点。修改参数不会启动电机。运行时修改仅保存在RAM，复位后恢复头文件配置值；本次未实现Flash保存或网口收发。既有START/END串口命令及Clamp_Execute接口仍可用。STATUS可查询当前生效参数。

以下示教流程作为可选方式；已在代码中配置端点时可直接查询状态并进行闭环测试。

工程 C:/D/AIRPORT，启动标识 AIRPORT clamp v5。两台电机方向0抓紧，方向1松开。

1. 接好两路传感器，确认STATUS两路valid=1，数值稳定，每路传感器对应同编号夹爪。
2. 用角度JOG将夹爪移到松开位置，停止后分别发送TEACH_START 1、TEACH_START 2。
3. 用方向0点动到抓紧位置，停止后分别发送TEACH_END 1、TEACH_END 2。首次用较短的端点间距。
4. 两端点录入后，程序自动确定抓紧时距离增减符号，无需CAL。STATUS应显示cal=1、endpoints=3、close_dir=0。端点间距须大于两倍tol（默认2 raw）。
5. 先逐轴RELEASE和GRIP验证，再使用不带轴号的双轴命令。

闭环使用连续脉冲输出，传感器只用于到位判断和故障保护。到位后停止，传感器无效或超时、无进展、总运动超时均停机。两轴独立到位，不保证同步轨迹。

JOG仍是无需传感器的角度点动；两端点录入后锁定JOG，重新示教先UNTEACH。位置是raw，尚未确认毫米倍率。端点保存在RAM，复位后需要重新示教。位置控制不检测抓力。编译及43项ARM仿真检查通过，尚未下载和实机闭环验证。

串口使用HEX发送，关闭自动循环，以下均已包含CRLF。

|命令|HEX|
|---|---|
|`STOP`|`53 54 4F 50 0D 0A`|
|`STATUS`|`53 54 41 54 55 53 0D 0A`|
|`TEACH_START 1`|`54 45 41 43 48 5F 53 54 41 52 54 20 31 0D 0A`|
|`TEACH_START 2`|`54 45 41 43 48 5F 53 54 41 52 54 20 32 0D 0A`|
|`JOG 1 0 360 200`|`4A 4F 47 20 31 20 30 20 33 36 30 20 32 30 30 0D 0A`|
|`JOG 2 0 360 200`|`4A 4F 47 20 32 20 30 20 33 36 30 20 32 30 30 0D 0A`|
|`TEACH_END 1`|`54 45 41 43 48 5F 45 4E 44 20 31 0D 0A`|
|`TEACH_END 2`|`54 45 41 43 48 5F 45 4E 44 20 32 0D 0A`|
|`GRIP 1`|`47 52 49 50 20 31 0D 0A`|
|`RELEASE 1`|`52 45 4C 45 41 53 45 20 31 0D 0A`|
|`GRIP 2`|`47 52 49 50 20 32 0D 0A`|
|`RELEASE 2`|`52 45 4C 45 41 53 45 20 32 0D 0A`|
|`GRIP`|`47 52 49 50 0D 0A`|
|`RELEASE`|`52 45 4C 45 41 53 45 0D 0A`|
|`UNTEACH 1`|`55 4E 54 45 41 43 48 20 31 0D 0A`|
|`UNTEACH 2`|`55 4E 54 45 41 43 48 20 32 0D 0A`|

网络命令入口保留在 Core/Inc/clamp_control.h：主循环调用Clamp_Execute，CLAMP_SET_START/CLAMP_SET_END设置绝对raw位置，CLAMP_GRIP/CLAMP_RELEASE执行，CLAMP_STOP停止；axis=0用于双轴抓取或松开。网口传输本次未实现。

到位保持：GRIP/RELEASE均为一次定位命令。进入容差带后停止脉冲并退出定位状态；后续传感器波动不会触发自动补偿。抓紧到位后由RELEASE命令返回起点。软件停止发脉冲，驱动器仍可保持励磁。默认容差0.2mm，可在头文件修改CLAMP_DEFAULT_TOLERANCE_MM。


## v7 振动下的一次到位控制
读数使用5点中位数，串口显示过滤后的毫米距离。朝命令目标单向移动，达到容差带或穿过目标后暂停脉冲，连续3个新的过滤读数确认到位；同一读数不会重复计数。确认期间若读数回到未到位侧，会继续运动。确认到位后退出定位，后续波动不会重启。不再因单次越位或小幅反向变化报OVERSHOOT/WRONG_DIRECTION；保留传感器无效/300ms过期、主循环失联、10秒无进展和30秒定位超时停机。默认用户当前容差0.5mm。持续错误的有效读数仍可能导致误判，滤波不能代替正确的光斑与目标板安装。编译与15项针对性ARM仿真检查通过，尚未实机验证。


## 2026-10-18 当前实机参数
两轴起点和终点由上位机通过TCP配置，默认起点275.0mm、终点130.0mm，容差0.5mm。定位超时CLAMP_MOVE_TIMEOUT_MS=180000；30秒内至少0.1mm预期进展，否则NO_PROGRESS。连续定位频率为1000Hz。传感器编号跟随电机：轴1 USART2，轴2 UART4。

