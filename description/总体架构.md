Motivation

  随着配送机器人逐渐进入校园、办公楼、医院和商场等半公共室内环境，人机共享通行空间的场景越来越常见。尤其是在狭窄走廊、电梯口、门口转角等通行余量有限的位置，机器人与行人迎面相遇时，往往会出现短暂僵持：机器人继续保持原路径或停在原地等待，而人也无法立即判断机器人接下来是准备通过、避让，还是停止不动。这类情况虽然不一定导致碰撞，但会明显影响通行效率，也会降低人对机器人行为的可理解性和信任感。
  现有配送机器人在导航层面通常更关注到达目标和避免碰撞，对于“如何以更符合人类直觉的方式与人共享狭窄空间”考虑相对不足。特别是在面对面会车场景中，如果机器人既不主动调整自身策略，也不向周围人传达其行为意图，就容易把通行决策的压力转移给人。对于人来说，这种不确定性本身就是一种交互负担。
  基于这一问题，我们希望设计并验证一种更主动、更具交互意识的机器人行为方式：当机器人在狭窄空间中检测到与行人的路径冲突时，不再简单停留或等待，而是主动选择退让，并在后退过程中结合周围障碍物信息进行安全避障。同时，机器人应能够以可感知的方式向人传达当前意图，例如表明自己正在让行或准备恢复通行，从而减少人对其行为的猜测成本。
Objective
Robotics Objective
做一个可演示、可比较的仿真 demo，验证「主动退让」比「原地等待」更合理，并符合老师的要求
- Key Results
  - 演示在2-3个场景中小车均成功避让了人，让人先通行，而后恢复自己的前进。
    - 可以考虑对比默认情况
  - 在演示中体现了小车轨迹规划的能力
- 核心交付物
  - 2-3个场景
  - ROS 控制代码
  - 小车原型
  - Presentation PPT + Report
HMI Objective
需求场景
场景理念
- 人类天然拥有更高的通行优先权
- 注意力状态不稳定，不能假设所有人都已经看到机器人
- 冲突通常是短时、局部和模糊的，而不是可被提前完全调度的
电梯场景（高优先级）
狭小空间单向交汇，机器人必须避让，而后进入
- WHEN 机器人想要进电梯，电梯人想往外出
- THEN 机器人退让到一侧
- WHEN 电梯内的人走出来了，离开了预定路径
- THEN 机器人继续向前
开阔路面场景（中优先级）
人没有看到机器人，机器人必须选择正确的避让方式
- WHEN 开阔路面上，人向机器人冲来
- THEN 机器人暂停后退规避
- WHEN 人仍然在向前
- THEN 机器人主动向侧边让开
- WHEN 机器人右边有人经过
- THEN 机器人只能向左边让开
建筑拐角（可做可不做）
- WHEN 建筑拐角走廊，人需要先过，遇到人靠近，机器人优先后退让人的场景
技术方案
技术栈
Ubuntu: 24.04 (Noble)
ROS2: Jazzy
Gazebo: Harmonic (LTS)
系统架构
模块定义（文件 base）
- hmi_behavior:
  - behavior 订阅topic、启动状态机、定时循环
  - state_machine 状态机 
  - scene_classifier 识别场景 
  - confilict_detector 判断是否存在冲突
  - robot_state 对机器人状态进行统一整理
    - 机器人的位置、朝向、速度
    - 人的位置、速度
    - 当前目标点
  - elevator_policy
  - open_area_policy
- hmi_bringup:
  - 总启动文件
- hmi_elements:
  - 机器小车
- hmi_world
  - world场景文件
- hmi_interfaces:
  - 自定义消息接口
状态机定义（待定）
协作 GPT 链接：https://chatgpt.com/share/69ef696e-1484-83a9-afe8-44d5e0e067d3
- Idle
  - 接收到任务 -> Navigate
- Navigate
  - 机器人正常前往目标点
  - 检测到冲突 -> ConflictAssess
- ConflictAssess
  - 无冲突/可继续前行 -> Navigate
  - 需要让行 -> YieldExecute
  - 暂时无法做出安全动作 -> YieldWait
- YieldExecute
  - If scene_type = elevator
    - -> ElevatorYield
  - If scene_type = open_area
    - -> OpenAreaYield
- ElevatorYield
  - 选择电梯门左侧等待区
  - 退让到等待区
  - -> YieldVerify
- OpenAreaYield
  - 暂停并开始检测
    - 若行人持续逼近
      - 右侧可用 -> 右让
      - 左侧可用 -> 左让
      - 否则 -> 后退/等待（等待最为安全）
- YieldVerify
  - 成功让出 -> YieldWait
  - 未成功解决 -> YieldExecute
  - 切换环境 -> ConflictAssess
- YieldWait
  - If scene_type = elevator
    - 电梯前矩形区域清空 -> Navigate
  - If scene_type = open_area
    - 以机器人为几何中心的矩形区域清空 -> Navigate
接口定义
ROS 接口与数据结构
- ActorState.msg：
  - std_msgs/Header header 时间戳
  - string actor_id
  - string actor_type
  - float32 x
  - float32 y
  - float32 yaw
  - float32 linear_x
  - float32 angular_z
  - bool is_moving 判断目标是否静止
- ObstacleState.msg：
  - std_msgs/Header header
  - string obstacle_id
  - float32 x
  - float32 y
  - float32 width
  - float32 length
  - bool is_static 目前默认true
- BehaviorState.msg：
  - std_msgs/Header header
  - string current_state
    - NormalMove / HumanDetected / YieldBackward / Waiting / Resume
  - string reason
    - human_close / obstacle_back / human_passed
  - float32 target_linear_x
  - float32 target_angular_z
话题流向
- /hmi/scene/robot_state
  - 发布方：场景/仿真同学
  - 订阅方：控制同学
  - 用途：把机器人当前位姿和速度发送给状态机
- /hmi/scene/human_state
  - 发布方：场景/仿真同学
  - 订阅方：控制同学
  - 用途：把人当前位置和速度发送给状态机
- /hmi/scene/obstacle_state
  - 发布方：场景/仿真同学
  - 订阅方：控制同学
  - 用途：把后方静态障碍物的位置发送给状态机，供后退避障判断
- /hmi/control/cmd_vel
  - 发布方：控制同学
  - 订阅方：机器人底盘/仿真执行侧
  - 用途：输出机器人前进、停止、后退的速度命令
- /hmi/control/behavior_state
  - 发布方：控制同学
  - 订阅方：集成同学、文档同学、后续可视化同学
  - 用途：输出当前状态机状态，便于调试和录屏展示
包编译方式：
  - 接口包：ament_cmake
  - 功能包：ament_python
其它对接要求，如场景
分工
- Robotics 架构设计
- ROS 代码开发
- 场景搭建
- Robotics Report + PowerPoint
- HMI Report + PowerPoint