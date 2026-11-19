# YOYO Skill 导入包

`honor-robotphone-base/SKILL.md` 是 Honor RobotPhone Base 的 YOYO Skill 源文件。
它将手机云台 API（`basicGimbal`、`operateGimbal`、`generateGimbal`）与底盘 BLE
控制分开：底盘通过 `speed <left> <right>` 实现双轮 PID 速度控制。

## 设备名称与连接

所有小车烧录同一份固件，BLE 广播名称为 `XIAO-Robot-Kit-` 加 BLE 地址后两字节
（如 `XIAO-Robot-Kit-3C7A`），机身标签印有该名称，手机蓝牙设置里也能看到。

本 Skill **不做 BLE 扫描**。首次动作请求时，Skill 请用户提供名称的后四位
（`XIAO-Robot-Kit-` 后面的四位），拼成完整名称并记住；之后所有指令都沿用这个
`device_name`，不再询问，只有用户明确说“换一台”或报出新后四位时才更换。
Service UUID `a2f39c10-7e4b-4d21-8a61-0f3bc9d15001` 与 Characteristic UUID
`a2f39c11-7e4b-4d21-8a61-0f3bc9d15001` 对所有小车固定不变，只有 `device_name`
随小车不同，因此小车改名后**不需要再手工编辑 SKILL.md**。

> 新固件默认即用 `XIAO-Robot-Kit-<BLE 地址后两字节>` 广播，无需逐台改名；想要正式编号时
> 才用 `btname set XIAO-Robot-Kit-01`（两位编号）覆盖并把标签贴成对应名称。
> `z1`~`w3` 专用固件各自使用不同的 UUID 和 `XIAO-MOTOR-*` 名称，不能用本 Skill 控制。

## 联动动作

`greeting`、`emotion_happy`、`emotion_scared`、`emotion_angry`、`reverse_park` 和
`dance_new_believer` 是云台-底盘复合动作：Skill 先打开手机云台，然后并行调用手机云台 API
与相应 BLE 命令。其中只有**倒车入库**和**跳舞**同时配音乐：

- 倒车入库：播放本地《请注意倒车8s》，云台按 5 个四拍小节模拟司机回头看后方；
- 跳舞：播放本地《New_Believer35》，云台按 19 个四拍小节编排摇摆/点头/摇头。

`greeting` 与三种情绪只联动云台、不配音乐，且只调用一次云台拟人化原子动作（点头/摇头/
抬头/转头），以贴近小车 0.5~3 秒的动作时长。跳舞用的《New_Believer35》前约 35 秒有声、
后约 300 秒静音，静音尾是故意的"模拟暂停"：手机 agent 下发指令后只有约 10 秒保活，无法在
音乐播完后执行关闭，因此整套跳舞联动必须一次性下发完毕、各方自行收尾，文件中不要裁剪或
替换该音乐。

手部另有 `wave_hand`、`handshake`、`open_arms`、`raise_both_arms` 和 `close_arms`；
旧 `greet`、`dance takehand` 和开环 `M|left|right|$` 不应用于当前预设动作。

## 生成上传包

荣耀开发者服务平台的“导入技能”要求 ZIP 文件。上传包的根目录必须直接包含 `SKILL.md`：

```bash
cd yoyo_skill/honor-robotphone-base
zip -j ../honor-robotphone-base-skill.zip SKILL.md
```

生成的 `yoyo_skill/honor-robotphone-base-skill.zip` 是本地发布产物，已由 Git 忽略。

## 平台导入与安装

1. 在荣耀开发者服务平台进入“管理中心 → 生态服务 → 智能体服务”。
2. 在“我的技能”中创建技能，选择“导入技能”，上传上述 ZIP 文件。
3. 在技能基础信息的允许工具（allowtools）中填写 `basicGimbal,operateGimbal,generateGimbal,playMusic,operateMusic,ble_send_command,skill_exit`。
4. 私有化发布后，在 RobotPhone 上使用同一账号登录 YOYO 助手，从“技能商店 → 我的技能 → 私有技能”安装。

> 即使 `SKILL.md` 已包含 `allowed-tools`，平台基础信息中的 allowtools 仍必须单独保存为
> `basicGimbal,operateGimbal,generateGimbal,playMusic,operateMusic,ble_send_command,skill_exit`；
> 缺少该后台配置可能导致手机端技能加载失败。本 Skill 不再使用 `bluetooth` 扫描工具。

## 首次真机验证

确认已烧录底盘固件、车轮悬空，并能从机身标签或手机蓝牙列表读出小车名称
（`XIAO-Robot-Kit-` 加后四位）后，依次测试。首次动作时 Skill 会询问名称后四位，
之后不再重复询问：

```text
@Honor RobotPhone Base 以 50 的速度前进 2 秒
@Honor RobotPhone Base 以 20 的速度前进 2 秒
@Honor RobotPhone Base 以 80 的速度前进 2 秒
@Honor RobotPhone Base 左轮 20、右轮 80，行驶 2 秒
@Honor RobotPhone Base 以 50 的速度前进 3 秒，然后以 50 的速度退后 3 秒
@Honor RobotPhone Base 打个招呼
@Honor RobotPhone Base 跳舞
@Honor RobotPhone Base 停止
```

固件已关闭全局自动停止。带时长的 PID 动作结束后，Skill 会显式发送 `stop`；未指定时长的
`speed` 命令会持续运动，直到用户发送“停止”或 BLE 断开。距离、角度和连续长时运动仍不应承诺
未经编码器闭环校准的运动精度。

对于“打个招呼”、三种情绪或“跳舞”，Skill 会先打开云台，再并行执行手机云台轨迹和 ESP32
对应命令；倒车入库与跳舞还会自然语言触发本地预置音乐播放。
请在平整、四周留有空间的地面上测试；所有复合动作完成后会自动停车，也可随时说“停止”。
