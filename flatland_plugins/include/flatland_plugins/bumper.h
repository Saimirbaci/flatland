/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  bumper.h
 * @brief   Bumper plugin
 * @author  Chunshang Li
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <flatland_msgs/Collisions.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <ros/ros.h>
#include <deque>
#include <random>
#include <string>
#include <utility>

#ifndef FLATLAND_PLUGINS_BUMPER_H
#define FLATLAND_PLUGINS_BUMPER_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class defines a bumper plugin that is used to publish the collisions
 * states of bodies in the model
 */
class Bumper : public ModelPlugin {
 public:
  struct ContactState {
    int num_count;  ///< stores number of times post solve is called
    double sum_normal_impulses[2];      ///< sum of impulses for averaging later
    double sum_tangential_impulses[2];  ///< sum of impulses for averaging later
    b2Vec2 points[2];  ///< Box2D collision points, max of 2 from Box2D
    b2Vec2 normal;  ///< normal of collision points, all points have same normal
    int normal_sign;  ///< for flipping direction of normal when necessary

    Body *body_A;      ///< the body of the model involved in the collision
    Body *body_B;      ///< the other body involved in the collision
    Entity *entity_B;  /// the entity the other body belongs to

    ContactState();  ///< initializes counters and sums
    void Reset();    ///< Reset counter and sums
  };

  std::string topic_name_;
  std::string world_frame_id_;           ///< name of the world frame id
  std::vector<Body *> excluded_bodies_;  ///< bodies to ignore
  /// whether to publish all collisions, or strictly adhere to update rate
  bool publish_all_collisions_;
  double update_rate_;  ///< rate to publish message at

  UpdateTimer update_timer_;  ///< for managing update rate

  /// For keeping track of contacts
  std::map<b2Contact *, ContactState> contact_states_;
  ros::Publisher collisions_publisher_;  ///< For publishing the collisions

  std::string fault_key_;           ///< registry component key (model/plugin)
  std::default_random_engine rng_;  ///< RNG for dropout / ghost-contact faults

  flatland_msgs::Collisions last_collisions_;  ///< last msg, for stuck/freeze
  bool last_collisions_valid_ = false;  ///< whether last_collisions_ is set

  /// Max buffered messages for the latency fault (bounds the delay queue).
  static constexpr size_t kMaxLatencyQueue = 256;
  /// Pending messages held by the latency fault, keyed by sim-time release.
  std::deque<std::pair<ros::Time, flatland_msgs::Collisions>> latency_queue_;

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

  /**
   * @brief Called when just after physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void AfterPhysicsStep(const Timekeeper &timekeeper) override;

  /**
   * @brief A method that is called for all Box2D begin contacts
   * @param[in] contact Box2D contact
   */
  void BeginContact(b2Contact *contact) override;

  /**
   * @brief A method that is called for all Box2D end contacts
   * @param[in] contact Box2D contact
   */
  void EndContact(b2Contact *contact) override;

  /*
   * @brief A method that is called for Box2D presolve
   * @param[in] contact Box2D contact
   * @param[in] oldManifold Manifold from the previous iteration
   */
  void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) override;

  /**
   * @brief Publish the collisions message, honouring the latency fault.
   * @details When the latency fault is active the message is buffered in a
   * sim-time-keyed FIFO and released once the Timekeeper clock reaches its
   * release instant (capture_time + severity*latency); the buffered message
   * keeps its original header.stamp. With no latency fault it releases the same
   * step, so the publish cadence is byte-for-byte identical to a clean run.
   * @param[in] timekeeper Object managing the simulation time
   * @param[in] collisions The message to (buffer and) publish
   * @param[in] enqueue If true, buffer the message; if false (dropout fired)
   * only drain already-buffered messages.
   */
  void PublishCollisions(const Timekeeper &timekeeper,
                         const flatland_msgs::Collisions &collisions,
                         bool enqueue);

  /**
   * @brief Release every buffered collisions message whose sim-time release has
   * arrived, in FIFO order. A no-op when the latency fault is inactive.
   * @param[in] timekeeper Object managing the simulation time
   */
  void DrainLatencyQueue(const Timekeeper &timekeeper);
};
};

#endif