/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser

#include <bondcpp/bond.hpp>

#ifdef _WIN32
#include <Rpc.h>
#else
#include <uuid/uuid.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

/**
 * @brief 生成一个唯一的字符串UUID
 * @return 返回生成的UUID字符串
 * @details 根据操作系统不同，使用不同的方式生成UUID
 * 该代码段实现了一个生成唯一UUID字符串的函数。根据操作系统不同，使用不同的方式生成UUID。对于Windows系统，使用系统API函数生成UUID，并将其转换为字符串格式；对于其他操作系统，同样使用系统API函数生成UUID，并将其转换为字符串格式。最终返回生成的UUID字符串。
 */
static std::string makeUUID() {
#ifdef _WIN32                  // 如果是Windows系统
  UUID uuid;                   // 定义UUID对象
  UuidCreate(&uuid);           // 使用系统API函数生成UUID
  unsigned char *str;          // 定义一个unsigned char类型的指针变量str
  UuidToStringA(&uuid, &str);  // 将UUID转换为字符串格式，并存储在str中
  std::string return_string(reinterpret_cast<char *>(
      str));  // 将str强制转换为char*类型，并创建一个std::string类型的对象return_string
  RpcStringFreeA(&str);  // 释放str指向的内存空间
  return return_string;  // 返回生成的UUID字符串
#else                    // 如果是其他操作系统
  uuid_t uuid;                   // 定义uuid_t类型的变量uuid
  uuid_generate_random(uuid);    // 使用系统API函数生成UUID
  char uuid_str[40];             // 定义一个长度为40的字符数组uuid_str
  uuid_unparse(uuid, uuid_str);  // 将UUID转换为字符串格式，并存储在uuid_str中
  return std::string(uuid_str);  // 将uuid_str创建为std::string类型的对象，并返回该对象
#endif
}

/**
 * @brief Bond 类的构造函数
 * @param topic bond 通信的主题名称
 * @param id bond 实例的唯一标识符
 * @param node_base NodeBaseInterface 的智能指针，提供节点基础接口
 * @param node_logging NodeLoggingInterface 的智能指针，提供节点日志接口
 * @param node_params NodeParametersInterface 的智能指针，提供节点参数接口
 * @param node_timers NodeTimersInterface 的智能指针，提供节点定时器接口
 * @param node_topics NodeTopicsInterface 的智能指针，提供节点话题接口
 * @param on_broken bond 断开连接时的回调函数
 * @param on_formed bond 建立连接时的回调函数
 * @details 构造函数初始化了 Bond 类的各个成员变量，并创建了一个 BondSM 对象和一个状态机对象 sm_。
 *          然后通过传入的 node_params
 * 智能指针判断是否需要禁用心跳超时，如果不需要则声明该参数并将其值设置为 false。 最后调用
 * setupConnections() 函数建立连接，并创建一个发布者和一个订阅者。
 */
Bond::Bond(
    const std::string &topic,
    const std::string &id,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    EventCallback on_broken,
    EventCallback on_formed)
    : node_base_(node_base),
      node_logging_(node_logging),
      node_timers_(node_timers),
      bondsm_(std::make_unique<BondSM>(this)),
      sm_(*bondsm_),
      topic_(topic),
      id_(id),
      instance_id_(makeUUID()),
      on_broken_(on_broken),
      on_formed_(on_formed),
      connect_timeout_(
          rclcpp::Duration::from_seconds(bond::msg::Constants::DEFAULT_CONNECT_TIMEOUT)),
      disconnect_timeout_(
          rclcpp::Duration::from_seconds(bond::msg::Constants::DEFAULT_DISCONNECT_TIMEOUT)),
      heartbeat_timeout_(
          rclcpp::Duration::from_seconds(bond::msg::Constants::DEFAULT_HEARTBEAT_TIMEOUT)),
      heartbeat_period_(
          rclcpp::Duration::from_seconds(bond::msg::Constants::DEFAULT_HEARTBEAT_PERIOD)),
      dead_publish_period_(
          rclcpp::Duration::from_seconds(bond::msg::Constants::DEAD_PUBLISH_PERIOD)) {
  if (!node_params->has_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM)) {
    node_params->declare_parameter(
        bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, rclcpp::ParameterValue(false));
  }

  disable_heartbeat_timeout_ =
      node_params->get_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM).as_bool();

  setupConnections();

  pub_ = rclcpp::create_publisher<bond::msg::Status>(
      node_params, node_topics, topic_, rclcpp::QoS(rclcpp::KeepLast(5)));

  sub_ = rclcpp::create_subscription<bond::msg::Status>(
      node_params, node_topics, topic_, rclcpp::QoS(100),
      std::bind(&Bond::bondStatusCB, this, std::placeholders::_1));
}

/*
上述代码是一个 Bond 类的实现。在 ROS2 中，Bond
是一种特殊的通信机制，用于检测两个节点之间的连接状态。当两个节点建立连接时，它们会创建一个 Bond
实例，并在 Bond 实例中注册回调函数。如果连接断开，回调函数将被调用。

该类有两个构造函数和一个析构函数。其中，第一个构造函数接受一个生命周期节点的指针，而第二个构造函数接受一个
ROS 节点的指针。这两个构造函数都会调用第三个构造函数，该构造函数初始化 Bond 对象的成员变量。

析构函数用于销毁 Bond 对象，并取消定时器。如果 Bond
对象未启动，则直接返回。否则，它会先断开连接，然后等待一段时间，以确保连接已经断开。如果 Bond
对象仍未断开，则输出错误信息。最后，它会取消所有定时器。
*/
/**
 * @brief Bond 构造函数，用于创建一个 Bond 对象
 * @param topic 用于创建 Bond 的 ROS 话题名称
 * @param id Bond 的唯一标识符
 * @param nh 生命周期节点的指针
 * @param on_broken 当 Bond 断开时调用的回调函数
 * @param on_formed 当 Bond 建立时调用的回调函数
 * @details 创建 Bond 对象，并初始化 Bond 对象的成员变量
 */
Bond::Bond(
    const std::string &topic,
    const std::string &id,
    rclcpp_lifecycle::LifecycleNode::SharedPtr nh,
    EventCallback on_broken,
    EventCallback on_formed)
    : Bond(
          topic,
          id,
          nh->get_node_base_interface(),
          nh->get_node_logging_interface(),
          nh->get_node_parameters_interface(),
          nh->get_node_timers_interface(),
          nh->get_node_topics_interface(),
          on_broken,
          on_formed) {}

/**
 * @brief Bond 构造函数，用于创建一个 Bond 对象
 * @param topic 用于创建 Bond 的 ROS 话题名称
 * @param id Bond 的唯一标识符
 * @param nh ROS 节点的指针
 * @param on_broken 当 Bond 断开时调用的回调函数
 * @param on_formed 当 Bond 建立时调用的回调函数
 * @details 创建 Bond 对象，并初始化 Bond 对象的成员变量
 */
Bond::Bond(
    const std::string &topic,
    const std::string &id,
    rclcpp::Node::SharedPtr nh,
    EventCallback on_broken,
    EventCallback on_formed)
    : Bond(
          topic,
          id,
          nh->get_node_base_interface(),
          nh->get_node_logging_interface(),
          nh->get_node_parameters_interface(),
          nh->get_node_timers_interface(),
          nh->get_node_topics_interface(),
          on_broken,
          on_formed) {}

/**
 * @brief Bond 析构函数，用于销毁 Bond 对象
 * @details 销毁 Bond 对象，并取消定时器
 */
Bond::~Bond() {
  if (!started_) {
    return;
  }
  breakBond();
  if (rclcpp::ok() && !waitUntilBroken(rclcpp::Duration(100ms))) {
    RCLCPP_DEBUG(
        node_logging_->get_logger(), "Bond failed to break on destruction %s (%s)", id_.c_str(),
        instance_id_.c_str());
  }

  publishingTimerCancel();
  deadpublishingTimerCancel();
  connectTimerCancel();
  heartbeatTimerCancel();
  disconnectTimerCancel();
}

/*
代码段中是一个 Bond 组件的实现，Bond 是 ROS2
中用于检测节点间连接状态的组件。该组件提供了一系列函数来设置连接超时时间、断开超时时间、心跳超时时间、心跳周期和死亡发布周期等参数，并在连接定时器到达设定的时间后执行
onConnectTimeout() 函数。

其中，setConnectTimeout() 函数用于设置连接超时时间，如果已经调用了 start()
函数，则无法设置超时时间；connectTimerReset() 函数用于重置连接定时器，在定时器到达设定的时间后执行
onConnectTimeout() 函数；setupConnections()
函数则用于设置连接超时时间、断开超时时间、心跳超时时间、心跳周期和死亡发布周期等参数。
*/
/**
 * @brief Bond 组件的连接设置函数，设置连接超时时间。
 * @param dur 连接超时时间，单位为秒。
 * @details 如果已经调用了 start() 函数，则无法设置超时时间。
 */
void Bond::setConnectTimeout(double dur) {
  if (started_) {
    RCLCPP_ERROR(node_logging_->get_logger(), "Cannot set timeouts after calling start()");
    return;
  }
  connect_timeout_ = rclcpp::Duration::from_seconds(dur);
}

/**
 * @brief Bond 组件的连接定时器重置函数。
 * @details 当连接定时器到达设定的时间后，会调用 onConnectTimeout() 函数。
 *          如果 connect_timer_reset_flag_ 和 started_ 都为 true，则执行 onConnectTimeout() 函数，
 *          并取消连接定时器，将 connect_timer_reset_flag_ 置为 false。
 */
void Bond::connectTimerReset() {
  // Callback function of connect timer
  auto connectTimerResetCallback = [this]() -> void {
    if (connect_timer_reset_flag_ && started_) {
      onConnectTimeout();
      connect_timer_->cancel();
      connect_timer_reset_flag_ = false;
    }  // flag is needed to have valid callback
  };
  // Connect timer started on node
  connect_timer_ = rclcpp::create_wall_timer(
      connect_timeout_.to_chrono<std::chrono::nanoseconds>(), connectTimerResetCallback, nullptr,
      node_base_.get(), node_timers_.get());
}

/**
 * @brief Bond
 * 组件的连接设置函数，设置连接超时时间、断开超时时间、心跳超时时间、心跳周期和死亡发布周期。
 * @details 分别设置连接超时时间、断开超时时间、心跳超时时间、心跳周期和死亡发布周期。
 *          这些时间都是 bond::msg::Constants 中定义的默认值。
 */
void Bond::setupConnections() {
  setConnectTimeout(bond::msg::Constants::DEFAULT_CONNECT_TIMEOUT);
  setDisconnectTimeout(bond::msg::Constants::DEFAULT_DISCONNECT_TIMEOUT);
  setHeartbeatTimeout(bond::msg::Constants::DEFAULT_HEARTBEAT_TIMEOUT);
  setHeartbeatPeriod(bond::msg::Constants::DEFAULT_HEARTBEAT_PERIOD);
  setDeadPublishPeriod(bond::msg::Constants::DEAD_PUBLISH_PERIOD);
}

/**
 * @brief 取消连接定时器
 * @param None
 * @details 如果已经存在连接定时器并且没有被取消，就取消它
 */
void Bond::connectTimerCancel() {
  if (connect_timer_ && !connect_timer_->is_canceled()) {
    connect_timer_->cancel();
  }
}

/**
 * @brief 设置断开连接超时时间
 * @param dur 超时时间（秒）
 * @details 如果已经调用了 start()，则无法设置超时时间，并打印错误信息
 */
void Bond::setDisconnectTimeout(double dur) {
  if (started_) {
    RCLCPP_ERROR(node_logging_->get_logger(), "Cannot set timeouts after calling start()");
    return;
  }
  disconnect_timeout_ = rclcpp::Duration::from_seconds(dur);
}

/**
 * @brief 重置断开连接定时器
 * @param None
 * @details 断开连接定时器的回调函数，如果标志位为真且已经启动，则执行 onDisconnectTimeout()
 * 函数并取消定时器
 */
void Bond::disconnectTimerReset() {
  auto disconnectTimerResetCallback = [this]() -> void {
    if (disconnect_timer_reset_flag_ && started_) {
      onDisconnectTimeout();
      disconnect_timer_->cancel();
      disconnect_timer_reset_flag_ = false;
    }  // flag is needed to have valid callback
  };
  // 在节点上启动断开连接定时器
  disconnect_timer_ = rclcpp::create_wall_timer(
      disconnect_timeout_.to_chrono<std::chrono::nanoseconds>(), disconnectTimerResetCallback,
      nullptr, node_base_.get(), node_timers_.get());
}

/**
 * @brief 取消断开连接定时器
 * @param None
 * @details 如果已经存在断开连接定时器并且没有被取消，就取消它
 */
void Bond::disconnectTimerCancel() {
  if (disconnect_timer_ && !disconnect_timer_->is_canceled()) {
    disconnect_timer_->cancel();
  }
}

/**
 * @brief 设置心跳超时时间
 * @param dur 超时时间（秒）
 * @details 如果已经调用了 start()，则无法设置超时时间，并打印错误信息
 */
void Bond::setHeartbeatTimeout(double dur) {
  if (started_) {
    RCLCPP_ERROR(node_logging_->get_logger(), "Cannot set timeouts after calling start()");
    return;
  }

  heartbeat_timeout_ = rclcpp::Duration::from_seconds(dur);
}

/*
这段代码是一个 bondcpp
组件的实现，其中包含了四个函数：heartbeatTimerReset()、heartbeatTimerCancel()、setHeartbeatPeriod()
和 publishingTimerReset()。

heartbeatTimerReset()
函数是重置心跳定时器的回调函数，如果未启动或禁用了心跳超时，则不执行任何操作；否则调用
onHeartbeatTimeout() 函数。在该函数中，使用 lambda 表达式定义了一个回调函数，并通过
rclcpp::create_wall_timer() 函数在节点上启动了一个心跳定时器。

heartbeatTimerCancel() 函数是取消心跳定时器的函数，如果心跳定时器存在且未被取消，则取消它。

setHeartbeatPeriod()
函数是设置心跳周期的函数，如果已经启动，则输出错误信息并返回；否则设置心跳周期为指定值。

publishingTimerReset() 函数是重置发布定时器的回调函数，调用 doPublishing()
函数。在该函数中，同样使用 lambda 表达式定义了一个回调函数，并通过 rclcpp::create_wall_timer()
函数在节点上启动了一个发布定时器。
*/
/**
 * @brief 重置心跳定时器的回调函数
 * @param 无
 * @details 如果未启动或禁用了心跳超时，则不执行任何操作；否则调用 onHeartbeatTimeout() 函数。
 */
void Bond::heartbeatTimerReset() {
  auto heartbeatTimerResetCallback = [this]() -> void {
    if (!started_ || disable_heartbeat_timeout_) {
      return;
    }

    onHeartbeatTimeout();
  };
  // 在节点上启动心跳定时器
  heartbeat_timer_ = rclcpp::create_wall_timer(
      heartbeat_timeout_.to_chrono<std::chrono::nanoseconds>(), heartbeatTimerResetCallback,
      nullptr, node_base_.get(), node_timers_.get());
}

/**
 * @brief 取消心跳定时器
 * @param 无
 * @details 如果心跳定时器存在且未被取消，则取消它。
 */
void Bond::heartbeatTimerCancel() {
  if (heartbeat_timer_ && !heartbeat_timer_->is_canceled()) {
    heartbeat_timer_->cancel();
  }
}

/**
 * @brief 设置心跳周期
 * @param dur 心跳周期，单位为秒
 * @details 如果已经启动，则输出错误信息并返回；否则设置心跳周期为指定值。
 */
void Bond::setHeartbeatPeriod(double dur) {
  if (started_) {
    RCLCPP_ERROR(node_logging_->get_logger(), "Cannot set timeouts after calling start()");
    return;
  }

  heartbeat_period_ = rclcpp::Duration::from_seconds(dur);
}

/**
 * @brief 重置发布定时器的回调函数
 * @param 无
 * @details 调用 doPublishing() 函数。
 */
void Bond::publishingTimerReset() {
  auto publishingTimerResetCallback = [this]() -> void { doPublishing(); };
  // 在节点上启动发布定时器
  publishing_timer_ = rclcpp::create_wall_timer(
      heartbeat_period_.to_chrono<std::chrono::nanoseconds>(), publishingTimerResetCallback,
      nullptr, node_base_.get(), node_timers_.get());
}

/*
publishingTimerCancel()：取消发布定时器。
setDeadPublishPeriod(double dur)：设置死亡发布周期。
deadpublishingTimerReset()：重置死亡发布定时器。
deadpublishingTimerCancel()：取消死亡发布定时器。
start()：启动 bond，连接定时器、发布定时器和断开连接定时器将被重置，并将 started_ 标志设置为 true。
*/
/**
 * @brief 取消发布定时器
 * @details 如果存在发布定时器且未被取消，则取消发布定时器
 */
void Bond::publishingTimerCancel() {
  if (publishing_timer_ && !publishing_timer_->is_canceled()) {
    publishing_timer_->cancel();
  }
}

/**
 * @brief 设置死亡发布周期
 * @param dur 死亡发布周期，单位为秒
 * @details 如果已经调用了 start() 函数，则无法设置超时时间
 */
void Bond::setDeadPublishPeriod(double dur) {
  if (started_) {
    RCLCPP_ERROR(node_logging_->get_logger(), "Cannot set timeouts after calling start()");
    return;
  }

  dead_publish_period_ = rclcpp::Duration::from_seconds(dur);
}

/**
 * @brief 重置死亡发布定时器
 * @details 死亡发布定时器的回调函数将在 bond 断开连接时发布数据
 */
void Bond::deadpublishingTimerReset() {
  //  callback function of dead publishing timer which will publish data when bond is broken
  auto deadpublishingTimerResetCallback = [this]() -> void { doPublishing(); };

  //  dead publishing timer started on node
  deadpublishing_timer_ = rclcpp::create_wall_timer(
      dead_publish_period_.to_chrono<std::chrono::nanoseconds>(), deadpublishingTimerResetCallback,
      nullptr, node_base_.get(), node_timers_.get());
}

/**
 * @brief 取消死亡发布定时器
 * @details 如果存在死亡发布定时器且未被取消，则取消死亡发布定时器
 */
void Bond::deadpublishingTimerCancel() {
  if (deadpublishing_timer_ && !deadpublishing_timer_->is_canceled()) {
    deadpublishing_timer_->cancel();
  }
}

/**
 * @brief 启动 bond
 * @details 连接定时器、发布定时器和断开连接定时器将被重置，并将 started_ 标志设置为 true
 */
void Bond::start() {
  connect_timer_reset_flag_ = true;
  connectTimerReset();
  publishingTimerReset();
  disconnectTimerReset();
  started_ = true;
}

/*
这段代码是在 ros2 项目中 bondcpp 组件相关的代码。其中 Bond 类提供了设置 Bond
组件已形成和已断开的回调函数以及等待 Bond 组件形成的功能。具体来说，setFormedCallback 和
setBrokenCallback 函数分别用于设置 Bond
组件已形成和已断开的回调函数，使用互斥锁保证线程安全；waitUntilFormed 函数用于等待 Bond
组件形成，使用 rclcpp::Rate 控制循环频率，使用 rclcpp::Duration 控制等待时间，如果超时或者 Bond
组件已形成，则退出循环并返回结果。
*/
/**
 * @brief 设置 Bond 组件已形成的回调函数
 * @param on_formed Bond 组件已形成的回调函数
 * @details 使用互斥锁保证线程安全，设置 Bond 组件已形成的回调函数
 */
void Bond::setFormedCallback(EventCallback on_formed) {
  std::unique_lock<std::mutex> lock(callbacks_mutex_);
  on_formed_ = on_formed;
}

/**
 * @brief 设置 Bond 组件已断开的回调函数
 * @param on_broken Bond 组件已断开的回调函数
 * @details 使用互斥锁保证线程安全，设置 Bond 组件已断开的回调函数
 */
void Bond::setBrokenCallback(EventCallback on_broken) {
  std::unique_lock<std::mutex> lock(callbacks_mutex_);
  on_broken_ = on_broken;
}

/**
 * @brief 等待 Bond 组件形成
 * @param timeout 超时时间
 * @return bool 返回 Bond 组件是否已形成
 * @details 使用 rclcpp::Rate 控制循环频率，使用 rclcpp::Duration 控制等待时间，
 *          如果超时或者 Bond 组件已形成，则退出循环并返回结果
 */
bool Bond::waitUntilFormed(rclcpp::Duration timeout) {
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  rclcpp::Time deadline(steady_clock.now() + timeout);
  rclcpp::Rate r(100);

  bool formed = false;
  while (!formed) {
    if (!rclcpp::ok()) {
      break;
    }
    rclcpp::Duration wait_time = rclcpp::Duration(100ms);
    if (timeout >= rclcpp::Duration(0.0s)) {
      wait_time = std::min(wait_time, deadline - steady_clock.now());
    }
    if (wait_time <= rclcpp::Duration(0.0s)) {
      break;  // 超时
    }
    r.sleep();

    if (!isStateWaitingForSister()) {
      formed = true;
    }
  }

  return formed;
}

/*
该代码段是一个 Bond 组件的相关代码，Bond 是 ROS2 中用于检测节点之间连接状态的组件。其中包含了等待
Bond 断开连接、检查 Bond 是否已经断开连接、检查 Bond 状态是否为 Alive 和检查 Bond 状态是否为
AwaitSisterDeath 四个函数。

waitUntilBroken 函数使用 while 循环等待 Bond 断开连接，如果超时或者 Bond
断开连接则跳出循环并返回结果。isBroken 函数用于检查 Bond 是否已经断开连接，isStateAlive 函数用于检查
Bond 状态是否为 Alive，isStateAwaitSisterDeath 函数用于检查 Bond 状态是否为 AwaitSisterDeath。其中
isStateDead 函数是 Bond 组件中的一个私有函数，用于检查 Bond 是否已经断开连接。
*/
/**
 * @brief 等待 Bond 断开连接
 * @param timeout 超时时间
 * @return bool 返回是否断开连接
 * @details 使用 while 循环等待 Bond 断开连接，如果超时或者 Bond 断开连接则跳出循环并返回结果
 */
bool Bond::waitUntilBroken(rclcpp::Duration timeout) {
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);          // 创建一个稳定时钟对象
  rclcpp::Time deadline(steady_clock.now() + timeout);  // 设置截止时间
  rclcpp::Rate r(100);                                  // 设置循环频率

  bool broken = false;                                  // 初始化断开状态为 false
  while (!broken) {                                     // 当未断开连接时执行循环
    if (!rclcpp::ok()) {  // 如果 ROS2 系统已经关闭，则跳出循环
      break;
    }
    rclcpp::Duration wait_time = rclcpp::Duration(100ms);  // 设置等待时间
    if (timeout >= rclcpp::Duration(0.0s)) {  // 如果设置了超时时间，则计算剩余等待时间
      wait_time = std::min(wait_time, deadline - steady_clock.now());
    }
    if (wait_time <= rclcpp::Duration(0.0s)) {  // 如果等待时间已经超过截止时间，则跳出循环
      break;                                    //  The deadline has expired
    }
    r.sleep();                                  // 等待一段时间

    if (isStateDead()) {                        // 检查 Bond 是否已经断开连接
      broken = true;  // 如果已经断开连接，则设置断开状态为 true
    }
  }

  return broken;  // 返回是否断开连接的状态
}

/**
 * @brief 检查 Bond 是否已经断开连接
 * @return bool 返回是否已经断开连接
 */
bool Bond::isBroken() { return isStateDead(); }

/**
 * @brief 检查 Bond 状态是否为 Alive
 * @return bool 返回 Bond 状态是否为 Alive
 */
bool Bond::isStateAlive() {
  std::unique_lock<std::mutex> lock(state_machine_mutex_);  // 创建互斥锁对象
  return sm_.getState().getId() == SM::Alive.getId();       // 返回 Bond 状态是否为 Alive
}

/**
 * @brief 检查 Bond 状态是否为 AwaitSisterDeath
 * @return bool 返回 Bond 状态是否为 AwaitSisterDeath
 */
bool Bond::isStateAwaitSisterDeath() {
  std::unique_lock<std::mutex> lock(state_machine_mutex_);  // 创建互斥锁对象
  return sm_.getState().getId() ==
         SM::AwaitSisterDeath.getId();  // 返回 Bond 状态是否为 AwaitSisterDeath
}

/**
 * @brief 判断 Bond 组件的状态是否为 Dead
 * @return 如果 Bond 组件的状态为 Dead，则返回 true；否则返回 false
 * @details 使用 std::unique_lock<std::mutex> 对 state_machine_mutex_ 进行加锁，然后通过
 * sm_.getState().getId() 获取当前状态的 id，并与 SM::Dead.getId() 进行比较，判断 Bond
 * 组件的状态是否为 Dead。
 */
bool Bond::isStateDead() {
  std::unique_lock<std::mutex> lock(state_machine_mutex_);
  return sm_.getState().getId() == SM::Dead.getId();
}

/**
 * @brief 判断 Bond 组件的状态是否为 WaitingForSister
 * @return 如果 Bond 组件的状态为 WaitingForSister，则返回 true；否则返回 false
 * @details 使用 std::unique_lock<std::mutex> 对 state_machine_mutex_ 进行加锁，然后通过
 * sm_.getState().getId() 获取当前状态的 id，并与 SM::WaitingForSister.getId() 进行比较，判断 Bond
 * 组件的状态是否为 WaitingForSister。
 */
bool Bond::isStateWaitingForSister() {
  std::unique_lock<std::mutex> lock(state_machine_mutex_);
  return sm_.getState().getId() == SM::WaitingForSister.getId();
}

/**
 * @brief 断开 Bond 组件的连接
 * @details 如果 Bond 组件的状态不为 Dead，则使用 std::unique_lock<std::mutex> 对
 * state_machine_mutex_ 进行加锁，然后调用 sm_.Die() 方法将状态设置为 Dead，并发布状态为 false
 * 的消息。最后调用 flushPendingCallbacks() 方法清空回调函数队列。
 */
void Bond::breakBond() {
  if (!isStateDead()) {
    {
      std::unique_lock<std::mutex> lock(state_machine_mutex_);
      sm_.Die();
    }
    publishStatus(false);
  }
  flushPendingCallbacks();
}

/**
 * @brief 处理连接超时事件
 * @details 使用 std::unique_lock<std::mutex> 对 state_machine_mutex_ 进行加锁，然后调用
 * sm_.ConnectTimeout() 方法将状态设置为 WaitingForSister，并调用 flushPendingCallbacks()
 * 方法清空回调函数队列。
 */
void Bond::onConnectTimeout() {
  {
    std::unique_lock<std::mutex> lock(state_machine_mutex_);
    sm_.ConnectTimeout();
  }
  flushPendingCallbacks();
}

/**
 * @brief 处理心跳超时事件
 * @details 使用 std::unique_lock<std::mutex> 对 state_machine_mutex_ 进行加锁，然后调用
 * sm_.HeartbeatTimeout() 方法将状态设置为 Dead，并调用 flushPendingCallbacks()
 * 方法清空回调函数队列。
 */
void Bond::onHeartbeatTimeout() {
  {
    std::unique_lock<std::mutex> lock(state_machine_mutex_);
    sm_.HeartbeatTimeout();
  }
  flushPendingCallbacks();
}

/*
这段代码是一个ROS2项目中bondcpp组件的相关代码。其中包含了三个函数，分别为onDisconnectTimeout、bondStatusCB和doPublishing。
onDisconnectTimeout函数用于更新状态机状态，并刷新挂起的回调函数，当连接超时时会调用此函数。
bondStatusCB函数用于接收bond状态消息，并根据情况发布状态消息。如果消息不是来自当前bond或者是来自本身，则忽略该消息。
doPublishing函数则根据当前状态机状态，决定是否发布状态消息，并在必要时取消定时器。
*/
/**
 * @brief Bond类的onDisconnectTimeout函数
 * @param 无
 * @details 当连接超时时，调用此函数以更新状态机状态，并刷新挂起的回调函数。
 */
void Bond::onDisconnectTimeout() {
  {
    std::unique_lock<std::mutex> lock(state_machine_mutex_);
    sm_.DisconnectTimeout();
  }
  flushPendingCallbacks();
}

/**
 * @brief Bond类的bondStatusCB函数
 * @param msg bond::msg::Status类型的消息
 * @details
 * 当接收到bond状态消息时，调用此函数以更新状态机状态，并根据情况发布状态消息。如果消息不是来自当前bond或者是来自本身，则忽略该消息。
 */
void Bond::bondStatusCB(const bond::msg::Status &msg) {
  if (!started_) {
    return;
  }

  //  Filters out messages from other bonds and messages from ourself
  if (msg.id == id_ && msg.instance_id != instance_id_) {
    if (sister_instance_id_.empty()) {
      sister_instance_id_ = msg.instance_id;
    }

    if (msg.active) {
      std::unique_lock<std::mutex> lock(state_machine_mutex_);
      sm_.SisterAlive();
    } else {
      {
        std::unique_lock<std::mutex> lock(state_machine_mutex_);
        sm_.SisterDead();
      }
      //  Immediate ack for sister's death notification
      if (sisterDiedFirst_) {
        publishStatus(false);
      }
    }
  }
  flushPendingCallbacks();
}

/**
 * @brief Bond类的doPublishing函数
 * @param 无
 * @details 根据当前状态机状态，决定是否发布状态消息，并在必要时取消定时器。
 */
void Bond::doPublishing() {
  if (isStateWaitingForSister() || isStateAlive()) {
    publishStatus(true);
  } else if (isStateAwaitSisterDeath()) {
    publishStatus(false);
  } else {  // SM::Dead
    publishingTimerCancel();
    deadpublishingTimerCancel();
  }
}

/*
该代码段是 ros2 项目中 bondcpp 组件相关的代码。其中，Bond
类包含了组件的状态信息和待处理的回调函数列表等成员变量，以及发布组件状态信息和清空待处理回调函数列表等成员函数。

publishStatus 函数用于发布组件的状态信息。首先创建一个 Status
消息，并填充消息头和组件状态信息等字段。然后通过 pub_->publish(msg) 发布该消息。

flushPendingCallbacks
函数用于清空待处理的回调函数列表。首先获取待处理的回调函数列表，然后清空该列表。最后依次执行回调函数。
*/
/**
 * @brief 发布 bond 组件的状态信息
 * @param active 组件是否处于活动状态
 * @details 创建一个 Status 消息，填充消息头和组件状态信息，然后发布该消息。
 */
void Bond::publishStatus(bool active) {
  bond::msg::Status msg;                                 // 创建一个 Status 消息
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);           // 创建一个时钟对象
  rclcpp::Time now = steady_clock.now();                 // 获取当前时间
  msg.header.stamp = now;                                // 填充消息头中的时间戳
  msg.id = id_;                                          // 填充组件 ID
  msg.instance_id = instance_id_;                        // 填充组件实例 ID
  msg.active = active;                                   // 填充组件状态
  msg.heartbeat_timeout = heartbeat_timeout_.seconds();  // 填充心跳超时时间
  msg.heartbeat_period = heartbeat_period_.seconds();    // 填充心跳周期
  pub_->publish(msg);                                    // 发布消息
}

/**
 * @brief 清空待处理的回调函数列表
 * @details 获取待处理的回调函数列表，然后清空列表。最后依次执行回调函数。
 */
void Bond::flushPendingCallbacks() {
  std::vector<EventCallback> callbacks;                   // 定义回调函数列表
  {
    std::unique_lock<std::mutex> lock(callbacks_mutex_);  // 创建互斥锁
    callbacks = pending_callbacks_;                       // 获取待处理的回调函数列表
    pending_callbacks_.clear();                           // 清空待处理的回调函数列表
  }

  for (size_t i = 0; i < callbacks.size(); ++i) {  // 依次执行回调函数
    callbacks[i]();
  }
}

}  //  namespace bond

/**
 * @brief BondSM类的Connected函数，表示bond连接成功时执行的操作。
 * @details 该函数会取消connectTimer，如果on_formed_为真，则将on_formed_添加到pending_callbacks_中。
 */
void BondSM::Connected() {
  b->connectTimerCancel();                           // 取消connectTimer
  if (b->on_formed_) {                               // 如果on_formed_存在
    std::unique_lock<std::mutex> lock(b->callbacks_mutex_);
    b->pending_callbacks_.push_back(b->on_formed_);  // 将on_formed_添加到pending_callbacks_中
  }
}

/**
 * @brief BondSM类的SisterDied函数，表示bond的sister死亡时执行的操作。
 * @details 将sisterDiedFirst_设置为true。
 */
void BondSM::SisterDied() {
  b->sisterDiedFirst_ = true;  // 将sisterDiedFirst_设置为true
}

/**
 * @brief BondSM类的Death函数，表示bond断开连接时执行的操作。
 * @details
 * 该函数会取消heartbeatTimer和disconnectTimer，如果on_broken_为真，则将on_broken_添加到pending_callbacks_中。
 */
void BondSM::Death() {
  b->heartbeatTimerCancel();                         // 取消heartbeatTimer
  b->disconnectTimerCancel();                        // 取消disconnectTimer
  if (b->on_broken_) {                               // 如果on_broken_存在
    std::unique_lock<std::mutex> lock(b->callbacks_mutex_);
    b->pending_callbacks_.push_back(b->on_broken_);  // 将on_broken_添加到pending_callbacks_中
  }
}

/**
 * @brief BondSM类的Heartbeat函数，表示bond发送心跳包时执行的操作。
 * @details 该函数会重置heartbeatTimer。
 */
void BondSM::Heartbeat() {
  b->heartbeatTimerReset();  // 重置heartbeatTimer
}

/**
 * @brief BondSM类的StartDying函数，表示bond开始断开连接时执行的操作。
 * @details
 * 该函数会取消heartbeatTimer，并将disconnect_timer_reset_flag_设置为true，重置disconnect_timer和deadpublishingTimer。
 */
void BondSM::StartDying() {
  b->heartbeatTimerCancel();               // 取消heartbeatTimer
  b->disconnect_timer_reset_flag_ = true;  // 将disconnect_timer_reset_flag_设置为true
  b->disconnect_timer_.reset();            // 重置disconnect_timer
  b->deadpublishingTimerReset();           // 重置deadpublishingTimer
}
