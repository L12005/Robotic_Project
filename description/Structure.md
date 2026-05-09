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

模块定义（按功能包划分）
- hmi_behavior:
  - behavior 订阅topic、启动状态机、定时循环
  - state_machine 状态机 
  - scene_classifier 识别场景 
  - conflict_detector 判断是否存在冲突
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
V2
基础参数定义：视行人自身加正前方 v*t 长度矩形为不可通行区域， v*t_2 退出区域
公共条件：当与行人的距离小于安全半径则 Wait
重要假设：小车速度大于行人，这个系统才能正常运行
- Idle
  - 未抵达目标点 -> Navigate
- Navigate （瞬间状态，寻找到达目标点的路径）
  - 路径可达 -> Forward
  - 路径不可达 -> Wait
  - 自身处于行人行进区域内  -> Conflict Avoiding Navigate
- Forward （前往目标点）
  - 原先路径不通或自身处于行人行进区域内 -> Navigate
  - 抵达目标点 -> Idle
- Conflict Avoiding Navigate （瞬间状态，寻找临时避障目标点并规划路径）
  - 若寻找到 -> Conflict Avoid
    - 方法为搜索自身周围固定半径一个圆上可达但离行人/多个行人最远的位置
  - 若未找到 -> Wait
- Conflict Avoid 寻路到临时避障目标点
  - 抵达后
    - 在退出区域内 -> Conflict Avoiding Navigate
    - 若不在退出区域内 -> Navigate
  - 当前路径不通 -> Conflict Avoiding Navigate
- Wait （等待）
  - 1s 后 -> Idle

接口定义与作用
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
    - human_close / human_passed
  - float32 target_linear_x
  - float32 target_angular_z

- nav_msgs/OccupancyGrid

1. OccupancyGrid 怎么用
  /hmi/scene/static_map 里放：
  墙体
  固定障碍物
  地图发布方先把静态障碍做膨胀，控制侧 A* 直接用这张膨胀后的图

2. ActorState 怎么用
  这个接口主要给“机器人”和“人”用。
  机器人 ActorState：
  x, y：A* 的起点
  yaw：路径跟踪时决定朝向控制
  linear_x, angular_z：调试用，或者判断机器人是否真的停下/在后退
  is_moving：可用于状态机确认“机器人是否已经停稳”
  
  （人）ActorState：
  x, y：做人机冲突检测
  yaw：确定“人前方禁行区”的方向
  linear_x：不用做动力学预测，但可以用来判断人是静止还是在前进
  is_moving：决定是继续等待还是准备恢复
  最关键的是：
  人不要写进 static_map，而是根据 ActorState 在运行时生成一层临时禁行区。
    可以在控制侧临时构造：
    人当前位置附近一个安全圆/矩形
    人朝向前方一段矩形禁行区/退出区域，然后把这层叠加到 static_map 的副本上，再跑 A*。

3. ObstacleState 怎么用

  x, y, width, length 用来在运行时往地图上叠加障碍块
  叠加时也要做同样的安全膨胀
  然后再跑 A*
  永久静态障碍：进 OccupancyGrid
  临时/可变障碍：走 ObstacleState

4. BehaviorState 怎么用
  这个不是规划输入，而是控制输出。
  它用于向录屏、调试、可视化和外部系统说明机器人当前为什么停、为什么退、什么时候恢复。

5. OccupancyGrid：
  用于地图定义
  该接口由官方包nav_msgs引入
  规范： 20格/m

话题流向
- /hmi/scene/robot_state
  - 发布方：场景/仿真同学
  - 订阅方：控制同学
  - 用途：把机器人当前位姿和速度发送给状态机
- /hmi/scene/human_state
  - 发布方：场景/仿真同学
  - 订阅方：控制同学
  - 用途：把人的当前位姿和速度发送给状态机，用于人机冲突检测与动态禁行区生成
- /hmi/scene/map_state
  - 发布方：场景同学
  - 订阅方：控制同学
  - 用途：低频发布地图信息给状态机，以供路径规划
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
