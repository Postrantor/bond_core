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

/** \author Stuart Glaser */

#ifndef BONDCPP__BOND_HPP_
#define BONDCPP__BOND_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "bond/msg/constants.hpp"
#include "bond/msg/status.hpp"
#include "bondcpp/BondSM_sm.hpp"

namespace bond {

/**
 * @brief
 * Bond类用于创建一个bond，以便监视另一个进程，并在它死亡时得到通知。同时，当你死亡时，它也会得到通知。
 */
class Bond {
public:
  using EventCallback = std::function<void(void)>;

  /**
   * @brief 委托构造函数
   *
   * @param topic 用于交换bond状态消息的话题。
   * @param id bond的ID，应与姐妹端使用的ID匹配。
   * @param node_base 节点基本接口
   * @param node_logging 节点日志接口
   * @param node_params 节点参数接口
   * @param node_timers 节点定时器接口
   * @param node_topics 节点话题接口
   * @param on_broken 当bond断开连接时调用的回调函数。
   * @param on_formed 当bond建立连接时调用的回调函数。
   */
  Bond(
      const std::string& topic,
      const std::string& id,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
      rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
      EventCallback on_broken = EventCallback(),
      EventCallback on_formed = EventCallback());

  /**
   * @brief 构造一个bond，但不连接
   *
   * @param topic 用于交换bond状态消息的话题。
   * @param id bond的ID，应与姐妹端使用的ID匹配。
   * @param nh 生命周期节点的共享指针。
   * @param on_broken 当bond断开连接时调用的回调函数。
   * @param on_formed 当bond建立连接时调用的回调函数。
   */
  Bond(
      const std::string& topic,
      const std::string& id,
      rclcpp_lifecycle::LifecycleNode::SharedPtr nh,
      EventCallback on_broken = EventCallback(),
      EventCallback on_formed = EventCallback());

  /**
   * @brief 构造一个bond，但不连接
   *
   * @param topic 用于交换bond状态消息的话题。
   * @param id bond的ID，应与姐妹端使用的ID匹配。
   * @param nh 节点的共享指针。
   * @param on_broken 当bond断开连接时调用的回调函数。
   * @param on_formed 当bond建立连接时调用的回调函数。
   */
  Bond(
      const std::string& topic,
      const std::string& id,
      rclcpp::Node::SharedPtr nh,
      EventCallback on_broken = EventCallback(),
      EventCallback on_formed = EventCallback());

  /**
   * @brief 析构函数，如果仍然连接，则断开bond。
   */
  ~Bond();

  /*
  该代码段定义了一些用于连接、断开连接、心跳检测和消息发布的计时器相关函数。其中，包括设置连接超时时间、断开连接超时时间、心跳超时时间、心跳周期、死亡发布周期等函数。这些函数可以帮助用户在
  ros2 项目中实现对连接状态的监测和控制，保证通信的稳定性和可靠性。
  */
  void setupConnections();

  double getConnectTimeout() const { return connect_timeout_.seconds(); }
  void setConnectTimeout(double dur);
  void connectTimerReset();
  void connectTimerCancel();

  double getDisconnectTimeout() const { return disconnect_timeout_.seconds(); }
  void setDisconnectTimeout(double dur);
  void disconnectTimerReset();
  void disconnectTimerCancel();

  double getHeartbeatTimeout() const { return heartbeat_timeout_.seconds(); }
  void setHeartbeatTimeout(double dur);
  void heartbeatTimerReset();
  void heartbeatTimerCancel();

  double getHeartbeatPeriod() const { return heartbeat_period_.seconds(); }
  void setHeartbeatPeriod(double dur);
  void publishingTimerReset();
  void publishingTimerCancel();

  double getDeadPublishPeriod() const { return dead_publish_period_.seconds(); }
  void setDeadPublishPeriod(double dur);
  void deadpublishingTimerReset();
  void deadpublishingTimerCancel();

  /*
    - `start()`：启动 Bond 并连接姐妹进程。
    - `setFormedCallback(EventCallback on_formed)`：设置组件形成时的回调函数。
    - `setBrokenCallback(EventCallback on_broken)`：设置组件断开时的回调函数。
    - `waitUntilFormed(rclcpp::Duration timeout =
    rclcpp::Duration(std::chrono::seconds(-1)))`：阻塞等待组件形成，最多等待 `timeout` 时间。
    - `waitUntilBroken(rclcpp::Duration timeout =
    rclcpp::Duration(std::chrono::seconds(-1)))`：阻塞等待组件断开，最多等待 `timeout` 时间。
    - `isBroken()`：指示 Bond 是否已经断开。
    - `breakBond()`：断开 Bond，通知另一个进程。
  */
  /**
   * @brief Bond 组件的启动函数，用于连接姐妹进程。
   */
  void start();

  /**
   * @brief 设置组件形成时的回调函数。
   * @param on_formed 形成时的回调函数。
   */
  void setFormedCallback(EventCallback on_formed);

  /**
   * @brief 设置组件断开时的回调函数。
   * @param on_broken 断开时的回调函数。
   */
  void setBrokenCallback(EventCallback on_broken);

  /**
   * @brief 阻塞等待组件形成，最多等待 'duration' 时间。
   *        假设节点在后台运行。
   * @param timeout 最长等待时间。如果为 -1，则此调用不会超时。
   * @return 如果已经形成 Bond，则返回 true。
   */
  bool waitUntilFormed(rclcpp::Duration timeout = rclcpp::Duration(std::chrono::seconds(-1)));

  /**
   * @brief 阻塞等待组件断开，最多等待 'duration' 时间。
   *        假设节点在后台运行。
   * @param timeout 最长等待时间。如果为 -1，则此调用不会超时。
   * @return 如果 Bond 已经断开，则返回 true，即使它从未形成过。
   */
  bool waitUntilBroken(rclcpp::Duration timeout = rclcpp::Duration(std::chrono::seconds(-1)));

  /**
   * @brief 指示 Bond 是否已经断开。
   * @return 如果 Bond 已经断开，则返回 true。
   */
  bool isBroken();

  /**
   * @brief 断开 Bond，通知另一个进程。
   */
  void breakBond();

  std::string getTopic() const { return topic_; }
  std::string getId() const { return id_; }
  std::string getInstanceId() const { return instance_id_; }

private:
  friend struct ::BondSM;

  void onConnectTimeout();
  void onHeartbeatTimeout();
  void onDisconnectTimeout();

  void bondStatusCB(const bond::msg::Status& msg);

  void doPublishing();
  void publishStatus(bool active);

  void flushPendingCallbacks();

  bool isStateAlive();
  bool isStateAwaitSisterDeath();
  bool isStateDead();
  bool isStateWaitingForSister();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;

  rclcpp::TimerBase::SharedPtr connect_timer_;
  rclcpp::TimerBase::SharedPtr disconnect_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr publishing_timer_;
  rclcpp::TimerBase::SharedPtr deadpublishing_timer_;

  std::mutex state_machine_mutex_;
  std::unique_ptr<BondSM> bondsm_;
  BondSMContext sm_;

  std::string topic_;
  std::string id_;
  std::string instance_id_;
  std::string sister_instance_id_;

  std::mutex callbacks_mutex_;
  std::vector<EventCallback> pending_callbacks_;
  EventCallback on_broken_;
  EventCallback on_formed_;

  bool sisterDiedFirst_{false};
  bool started_{false};
  bool connect_timer_reset_flag_{false};
  bool disconnect_timer_reset_flag_{false};
  bool deadpublishing_timer_reset_flag_{false};
  bool disable_heartbeat_timeout_{false};

  rclcpp::Duration connect_timeout_;
  rclcpp::Duration disconnect_timeout_;
  rclcpp::Duration heartbeat_timeout_;
  rclcpp::Duration heartbeat_period_;
  rclcpp::Duration dead_publish_period_;

  rclcpp::Subscription<bond::msg::Status>::SharedPtr sub_;
  rclcpp::Publisher<bond::msg::Status>::SharedPtr pub_;
};
}  // namespace bond

// Internal use only
// 定义了一个名为BondSM的结构体，用于处理bond状态机的不同事件
struct BondSM {
  explicit BondSM(bond::Bond* b_) : b(b_) {}

  void Connected();   // 处理连接成功事件。
  void SisterDied();  // 处理sister死亡事件。
  void Death();       // 处理bond死亡事件。
  void Heartbeat();   // 处理心跳事件。
  void StartDying();  // 开始死亡过程。

private:
  bond::Bond* b;
};

#endif  // BONDCPP__BOND_HPP_
