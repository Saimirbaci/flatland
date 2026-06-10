#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/noise_model.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <deque>
#include <random>
#include <string>
#include <utility>

#ifndef FLATLAND_PLUGINS_GPS_H
#define FLATLAND_PLUGINS_GPS_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class simulates a GPS receiver in Flatland
 */
class Gps : public ModelPlugin {
 public:
  std::string topic_;     ///< topic name to publish the GPS fix
  std::string frame_id_;  ///< GPS frame ID
  Body *body_;            ///< body the simulated GPS antenna attaches to
  Pose origin_;           ///< GPS sensor frame w.r.t the body
  double ref_lat_rad_;  ///< latitude in radians corresponding to (0, 0) in map
                        /// frame
  double ref_lon_rad_;  ///< longitude in radians corresponding to (0, 0) in map
                        /// frame
  double ref_ecef_x_;   ///< ECEF coordinates of reference lat and lon at zero
                        /// altitude
  double ref_ecef_y_;   ///< ECEF coordinates of reference lat and lon at zero
                        /// altitude
  double ref_ecef_z_;   ///< ECEF coordinates of reference lat and lon at zero
                        /// altitude
  double update_rate_;  ///< GPS fix publish rate
  bool broadcast_tf_;   ///< whether to broadcast laser origin w.r.t body

  static double WGS84_A;   ///< Earth's major axis length
  static double WGS84_E2;  ///< Square of Earth's first eccentricity

  ros::Publisher fix_publisher_;             ///< GPS fix topic publisher
  tf::TransformBroadcaster tf_broadcaster_;  ///< broadcast GPS frame
  geometry_msgs::TransformStamped gps_tf_;   ///< tf from body to GPS frame
  sensor_msgs::NavSatFix gps_fix_;           ///< message for publishing output
  UpdateTimer update_timer_;                 ///< for controlling update rate

  Eigen::Matrix3f m_body_to_gps_;  ///< tf from body to GPS

  std::string fault_key_;           ///< registry component key (model/plugin)
  std::default_random_engine rng_;  ///< RNG for dropout / noise faults

  /// Optional calibrated, context-conditioned baseline noise model. GPS has no
  /// legacy baseline noise, so the legacy path adds nothing (fallback std 0).
  std::shared_ptr<flatland_server::NoiseModel> noise_model_;
  double sensor_age_hours_ = 0.0;  ///< configured sensor age (noise context)
  bool use_noise_model_ = false;   ///< true when a calibrated model is loaded

  sensor_msgs::NavSatFix last_fix_;  ///< last fix, for the stuck/freeze fault
  bool last_fix_valid_ = false;      ///< whether last_fix_ holds a fix

  /// Max buffered fixes for the latency fault (bounds the delay queue).
  static constexpr size_t kMaxLatencyQueue = 256;
  /// Pending fixes held by the latency fault, keyed by sim-time release.
  std::deque<std::pair<ros::Time, sensor_msgs::NavSatFix>> latency_queue_;

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
   * @brief Helper function to extract the paramters from the YAML Node
   * @param[in] config Plugin YAML Node
   */
  void ParseParameters(const YAML::Node &config);

  /**
   * @brief Method to compute ECEF coordinates of reference
   * latitude and longitude, assuming zero altitude
   */
  void ComputeReferenceEcef();

  /**
   * @brief Method that updates the current state of the GPS fix output
   * for publishing
   */
  void UpdateFix();

  /**
   * @brief Publish the current fix, honouring the latency fault.
   * @details When the latency fault is active the fix is buffered in a
   * sim-time-keyed FIFO and released once the Timekeeper clock reaches its
   * release instant (capture_time + severity*latency); the buffered message
   * keeps its original header.stamp. With no latency fault the fix releases the
   * same step, so the publish cadence is byte-for-byte identical to a clean
   * run. Matured buffered fixes are always drained, even on a dropped step.
   * @param[in] timekeeper Object managing the simulation time
   * @param[in] enqueue If true, buffer the current fix; if false (dropout
   * fired) only drain already-buffered fixes.
   */
  void PublishFix(const Timekeeper &timekeeper, bool enqueue);
};
}

#endif  // FLATLAND_PLUGINS_GPS_H
