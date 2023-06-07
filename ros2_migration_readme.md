---
tip: translate by baidu@2023-06-07 08:43:13
...

This document contains the overview of the bond-core package, build and test procedure to test package and the changes made with respect to migration of bond-core from ROS to ROS2. Later current limitations are also listed down.

> 本文件包含结合核心包的概述、测试包的构建和测试程序，以及结合核心从 ROS 迁移到 ROS2 所做的更改。后面列出了当前的限制。

## Overview

A bond allows two processes, A and B, to know when the other has terminated, either cleanly or by crashing. The bond remains connected until it is either broken explicitly or until a heartbeat times out. Constructor of object is as follows,

> 键允许 A 和 B 两个进程知道另一个进程何时终止，无论是干净的还是崩溃的。这种联系一直保持着联系，直到它被明确地打破，或者直到心跳停止。对象的构造函数如下，

```cpp
bond::Bond::Bond(
	const std::string & topic, const std::string & id, rclcpp::Node::SharedPtr nh,
	std::function<void(void)> on_broken,
	std::function<void(void)> on_formed)
```

Parameters

```
		topic      : The topic used to exchange the bond status messages.
		id	       : The ID of the bond, which should match the ID used on the sister's end.
		nh         : The node used to from bond
		on_broken  : Callback that will be called when the bond is broken.
		on_formed  : Callback that will be called when the bond is formed.
```

Bond-core (CPP) is have following functionalities as per ROS.

> 根据 ROS，粘结芯（CPP）具有以下功能。

    - start() : Starts the bond and connects to the sister process.
    - setFormedCallback(std::function<void(void)> on_formed) : Sets the formed callback.
    - setBrokenCallback(std::function<void(void)> on_broken) : Sets the broken callback.
    - waitUntilFormed(rclcpp::Duration timeout) : Blocks until the bond is formed for at most 'duration', timeout Maximum duration to wait.
    - waitUntilBroken(rclcpp::Duration timeout) : Blocks until the bond is broken for at most 'duration', timeout Maximum duration to wait.
    - isBroken() : Indicates if the bond is broken.
    - breakBond() : Breaks the bond, notifying the other process.
    - getTopic() : Gets topic name.
    - getId() : Gets ID.
    - getInstanceId() : Gets instant ID.
    - getConnectTimeout() : Gets connect timeout in nanoseconds.
    - setConnectTimeout(double dur) : Sets connect timeout in seconds.
    - getDisconnectTimeout(): Gets disconnect timeout in nanoseconds.
    - setDisconnectTimeout(double dur) : Sets disconnect timeout in seconds.
    - getHeartbeatTimeout() : Gets heatrbeat timeout in nanoseconds.
    - setHeartbeatTimeout(double dur) : Sets heartbeat timeout in seconds.
    - getHeartbeatPeriod() : Gets heartbeat period (publishing period) in naoseconds.
    - setHeartbeatPeriod(double dur) : Sets heartbeat period (publishing period) in seconds.
    - getDeadPublishPeriod() : Gets dead publish period in nanoseconds.
    - setDeadPublishPeriod(double dur) : Sets dead publish period in seconds.

> - start（）：启动绑定并连接到姐妹进程。
> - setFormedCallback（std:：function ＜ void（void）＞ on_formed）：设置已形成的回调。
> - setBrokenCallback（std:：function ＜ void（void）＞ on_brook）：设置中断的回调。
> - waitUntilFormed（rclcpp:：Duration timeout）：在债券形成之前阻止最多“Duration”，timeout 等待的最大持续时间。
> - waitUntilBroken（rclcpp:：Duration timeout）：阻止绑定，直到绑定断开最多“Duration”，timeout 等待的最大持续时间。
> - isBroken（）：指示键是否已断开。
> - breakBond（）：断开连接，通知其他进程。
> - getTopic（）：获取主题名称。
> - getId（）：获取 ID。
> - getInstanceId（）：获取即时 ID。
> - getConnectTimeout（）：获取以纳秒为单位的连接超时。
> - setConnectTimeout（double-dur）：以秒为单位设置连接超时。
> - getDisconnectTimeout（）：获取以纳秒为单位的断开超时。
> - setDisconnectTimeout（double-dur）：以秒为单位设置断开连接超时。
> - getHeartbeatTimeout（）：以纳秒为单位获取 heartbeat 超时。
> - setHeartbeatTimeout（double-dur）：以秒为单位设置检测信号超时。
> - getHeartbeatPeriod（）：获取以 naoseconds 为单位的心跳周期（发布周期）。
> - setHeartbeatPeriod（double-dur）：以秒为单位设置检测信号周期（发布周期）。
> - getDeadPublishPeriod（）：获取以纳秒为单位的死发布周期。
> - setDeadPublishPeriod（双 dur）：设置以秒为单位的死发布周期。

## Build proccedure and testing

```
1. Get pacakge at local system
				1.1 # mkdir -p ros2_ws_bond/src
				1.2 # cd ros2_ws_bond/src
				1.3 # git clone git@github.com:kishornaik10/bond_core.git
	or git clone https://github.com/kishornaik10/bond_core.git
	(checkout for ros2-devel branch)
				1.4 # source /opt/ros/crystal/setup.sh
2. Build the package
					2.1 # cd ../
					2.2 # colcon build
3. Do the test
					3.1 # colcon test
4. The executables are generated follow the below steps to run tests.
				4.1 source local setup
					# source install/local_setup.sh
				4.2 run the executables.
./install/test_bond/lib/test_bond/exercise_bond_cpp_exc
./install/test_bond/lib/test_bond/test_callbacks_cpp_exc
```

## ROS2 Migration changes

The basic concept and design are same as ROS. All changes for migration have been done as per Migration guide.

> 基本概念和设计与 ROS 相同。迁移的所有更改都已按照迁移指南进行。

1. Node is passed to a constructor of bond class.
2. The timeout class is removed and steady timer is replaced by wall timer.
3. a) For each timer, timeout time is set.
   b) Callback function is written in such a way that it will be called on valid request, so flag is used.
4. waitUntilFormed and waitUntilBroken functions modified, locking system removed to handle callback of node as Callback Queue is not available in ROS2.

> 1. 节点被传递给 bond 类的构造函数。
> 2. 超时类被删除，稳定计时器被墙壁计时器取代。
> 3. a）对于每个定时器，设置超时时间。
>    b） 回调函数是以这样一种方式编写的，即它将在有效请求时被调用，所以使用了 flag。
> 4. 修改了 waitUntilFormed 和 waitUntlBroken 函数，删除了锁定系统来处理节点的回调，因为回调队列在 ROS2 中不可用。

## Limitations

1. Work around is done for timer.
2. CallbackQueue is not available hence function related to Callback Queue commented out.
3. Dead publising period and timer is not used.
4. bondpy needs to be migrate.

> 1. 计时器已完成修复。
> 2. 回调队列不可用，因此与回调队列相关的函数被注释掉。
> 3. 没有使用死出版期和定时器。
> 4. 需要迁移 bondpy。
