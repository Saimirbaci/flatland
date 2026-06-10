/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 world.h
 * @brief	 Definition for the simulation world
 * @author Joseph Duchesne
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

#ifndef FLATLAND_SERVER_WORLD_H
#define FLATLAND_SERVER_WORLD_H

#include <Box2D/Box2D.h>
#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/interactive_marker_manager.h>
#include <flatland_server/layer.h>
#include <flatland_server/model.h>
#include <flatland_server/plugin_manager.h>
#include <flatland_server/surface_friction_field.h>
#include <flatland_server/timekeeper.h>
#include <ros/ros.h>

#include <map>
#include <string>
#include <vector>

namespace flatland_server {

/**
 * This class defines a world in the simulation. A world contains layers
 * that can represent environments at multiple levels, and models which are
 * can be robots or obstacles.
 */
class World : public b2ContactListener {
 public:
  boost::filesystem::path world_yaml_dir_;  ///< directory containing world file
  b2World* physics_world_;                  ///< Box2D physics world
  b2Vec2 gravity_;  ///< Box2D world gravity, always (0, 0)
  std::map<std::vector<std::string>, Layer*>
      layers_name_map_;           ///< map of all layers and thier name
  std::vector<Layer*> layers_;    ///< list of layers
  std::vector<Model*> models_;    ///< list of models
  CollisionFilterRegistry cfr_;   ///< collision registry for layers and models
  PluginManager plugin_manager_;  ///< for loading and updating plugins
  bool service_paused_;  ///< indicates if simulation is paused by a service
                         /// call or not
  InteractiveMarkerManager
      int_marker_manager_;  ///< for dynamically moving models from Rviz
  int physics_position_iterations_;  ///< Box2D solver param
  int physics_velocity_iterations_;  ///< Box2D solver param
  int physics_substeps_;     ///< number of fixed sub-steps Box2D integrates per
                             /// outer step (>=1). Splits the visible step_size
                             /// into finer dt for contact stability at AGV
                             /// mass/speed without changing the published clock
                             /// or plugin step cadence.
  bool continuous_physics_;  ///< toggles Box2D continuous collision detection
                             ///(CCD). Keeps fast/thin bodies from tunnelling
                             /// through walls; disable to trade realism for
                             /// throughput.
  SurfaceFrictionField
      surface_friction_;  ///< per-region surface friction
                          /// multiplier field (wet patches,
                          /// spills, ramps). Disabled (factor
                          /// 1.0 everywhere) unless the world
                          /// YAML has a surface_friction block.

  ros::NodeHandle
      nh_;  ///< node handle owning the diagnostic-overlay publishers
  std::vector<ros::Publisher>
      layer_occupancy_pubs_;  ///< latched per-layer OccupancyGrid publishers
                              /// (kept alive so late subscribers still latch)
  ros::Publisher
      friction_regions_pub_;  ///< latched friction-region MarkerArray

  /**
   * @brief Constructor for the world class. All data required for
   * initialization should be passed in here
   */
  World();

  /**
   * @brief Destructor for the world class
   */
  ~World();

  /**
   * @brief trigger world update include all physics and plugins
   * @param[in] timekeeper The time keeping object
   */
  void Update(Timekeeper& timekeeper);

  /**
   * @brief Box2D inherited begin contact
   * @param[in] contact Box2D contact information
   */
  void BeginContact(b2Contact* contact) override;

  /**
   * @brief Box2D inherited end contact
   * @param[in] contact Box2D contact information
   */
  void EndContact(b2Contact* contact) override;

  /**
   * @brief Box2D inherited presolve
   * @param[in] contact Box2D contact information
   * @param[in] oldManifold The manifold from the previous timestep
   */
  void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);

  /**
   * @brief Box2D inherited pre solve
   * @param[in] contact Box2D contact information
   * @param[in] impulse The calculated impulse from the collision resolute
   */
  void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);

  /*
   * @brief Load world plugins
   * @param[in] world_plugin_reader, readin the info about the plugin
   * @param[in] world, the world where the plugin will be applied to
   * @param[in] world config, the yaml reader of world.yaml
   */
  void LoadWorldPlugins(YamlReader& world_plugin_reader, World* world,
                        YamlReader& world_config);
  /**
   * @brief load layers into the world. Throws YAMLException.
   * @param[in] layers_reader Yaml reader for node that has list of layers
   */
  void LoadLayers(YamlReader& layers_reader);

  /**
   * @brief load models into the world. Throws YAMLException.
   * @param[in] layers_reader Yaml reader for node that has a list of models
   */
  void LoadModels(YamlReader& models_reader);

  /**
   * @brief load models into the world. Throws YAMLException.
   * @param[in] model_yaml_path Relative path to the model yaml file
   * @param[in] ns Namespace of the robot
   * @param[in] name Name of the model
   * @param[in] pose Initial pose of the model in x, y, yaw
   */
  void LoadModel(const std::string& model_yaml_path, const std::string& ns,
                 const std::string& name, const Pose& pose);

  /**
   * @brief remove model with a given name
   * @param[in] name The name of the model to remove
   */
  void DeleteModel(const std::string& name);

  /**
   * @brief move model with a given name
   * @param[in] name The name of the model to move
   * @param[in] pose The desired new pose of the model
   */
  void MoveModel(const std::string& name, const Pose& pose);

  /**
   * @brief set the paused state of the simulation to true
   */
  void Pause();

  /**
   * @brief set the paused state of the simulation to false
   */
  void Resume();

  /**
   * @brief toggle the paused state of the simulation
   */
  void TogglePaused();

  /**
   * @brief returns true if service_paused_ is true or an interactive marker is
   * currently being dragged
   */
  bool IsPaused();

  /**
   * @brief factory method to create a instance of the world class. Cleans all
   * the inputs before instantiation of the class. TThrows YAMLException.
   * @param[in] yaml_path Path to the world yaml file
   * @param[in] seed run seed used to deterministically seed all RNG sources
   *            (plugins, world_random_wall, the Lua preprocessor). <0 means
   *            nondeterministic, falling back to the world YAML
   * properties.seed.
   * @return pointer to a new world
   */
  static World* MakeWorld(const std::string& yaml_path, int seed = -1);

  /**
   * @brief Sample the surface friction multiplier at a world point.
   *
   * Returns a smoothly-varying, bounded multiplier (1.0 = nominal/dry) that
   * plugins scale their friction coefficients by, so a body loses traction
   * continuously over wet patches/spills and recovers on dry ground. Worlds
   * without a surface_friction block always return 1.0.
   *
   * @param[in] point Query point in world coordinates [m]
   * @return Surface friction multiplier at the point
   */
  double GetSurfaceFrictionFactor(const b2Vec2& point) const {
    return surface_friction_.GetFrictionFactor(point);
  }

  /**
   * @brief Activate (or update) a circular low-friction spill region at run
   * time.
   *
   * Hook used by the FaultInjector's `spill` environment fault to make a
   * low-traction patch appear mid-run. Forwards to the surface-friction field's
   * analytic overlay; the drive plugins already sample
   * GetSurfaceFrictionFactor() per wheel, so the robot loses traction over the
   * spill with zero change when none is active. Re-calling with the same @p id
   * updates the region in place (so severity can ramp the multiplier).
   *
   * @param[in] id Stable spill id (e.g. the fault id)
   * @param[in] center Spill centre in world coordinates [m]
   * @param[in] radius Spill outer radius [m]
   * @param[in] mu Slipperiest multiplier at the spill core
   */
  void AddSpillRegion(const std::string& id, const b2Vec2& center,
                      double radius, double mu) {
    surface_friction_.AddCircularRegion(id, center, radius, mu);
  }

  /**
   * @brief Deactivate a spill region previously added with @p id (no-op if
   *        absent). Called on the fault's end-edge so grip recovers.
   */
  void RemoveSpillRegion(const std::string& id) {
    surface_friction_.RemoveCircularRegion(id);
  }

  /**
   * @brief Wire the process-wide NoiseContextProvider to this world.
   *
   * Sets the ambient lighting scalar from the optional world-YAML
   * `noise_context` block (default 1.0) and installs a surface sampler that
   * buckets GetSurfaceFrictionFactor() into a surface_id, so sensor/drive noise
   * models can condition on the surface under the robot. A null reader leaves
   * the provider at its defaults, so worlds without the block are unchanged.
   *
   * @param[in] reader YamlReader for the optional `noise_context` subnode.
   */
  void ConfigureNoiseContext(YamlReader& reader);

  /**
   * @brief Publish debug visualizations for everything
   * @param[in] update_layers since layers are pretty much static, this
   * parameter is used to skip updating layers
   */
  void DebugVisualize(bool update_layers = true);

  /**
   * @brief Publish the static diagnostic overlays once, on latched topics.
   *
   * Advertises and publishes one nav_msgs/OccupancyGrid per image-based layer
   * (`/flatland_server/layers/<name>/occupancy`) and, when a surface_friction
   * block is configured, a visualization_msgs/MarkerArray of friction regions
   * (`/flatland_server/debug/friction_regions`). Everything is latched and
   * published exactly once at world-load time, so this adds no per-step cost
   * and late subscribers (rviz, the flatland_viz Diagnostic Layers panel) still
   * receive the data. Safe to call once after the world is built; calling it
   * again simply re-advertises and re-publishes.
   */
  void PublishDiagnostics();
};
};  // namespace flatland_server
#endif  // FLATLAND_SERVER_WORLD_H
