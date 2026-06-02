/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   diagnostic_layer_panel.h
 * @brief  Dockable panel of per-overlay show/hide checkboxes for flatland_viz
 * @author Saimir Baci
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Avidbots Corp.
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

#ifndef FLATLAND_VIZ_DIAGNOSTIC_LAYER_PANEL_H
#define FLATLAND_VIZ_DIAGNOSTIC_LAYER_PANEL_H

#include <QDockWidget>
#include <QString>
#include <map>

class QCheckBox;
class QVBoxLayout;

namespace rviz {
class Display;
}

/**
 * @brief A dockable panel that lists the diagnostic overlays as checkboxes.
 *
 * Each entry binds a checkbox to an rviz::Display; toggling the box flips the
 * display's enabled state (show/hide) without touching any data flow. The panel
 * owns no displays — it holds non-owning pointers handed to it by FlatlandViz,
 * which creates and outlives the displays in its VisualizationManager.
 */
class DiagnosticLayerPanel : public QDockWidget {
  Q_OBJECT

 public:
  /**
   * @brief Construct the panel and its (initially empty) checkbox column.
   * @param parent Parent widget
   */
  explicit DiagnosticLayerPanel(QWidget* parent = nullptr);

  /**
   * @brief Add a labelled overlay toggle bound to an rviz Display.
   * @param label Human-readable layer name shown beside the checkbox
   * @param display The display whose visibility the checkbox controls (may be
   *                null, in which case the checkbox is disabled)
   * @param initially_enabled Initial checked state; also applied to the display
   */
  void addLayer(const QString& label, rviz::Display* display,
                bool initially_enabled);

  /**
   * @brief Add a checkbox not bound to a single display, for the caller to wire
   *        up itself (e.g. an aggregate toggle over many displays).
   * @param label Human-readable layer name shown beside the checkbox
   * @param initially_enabled Initial checked state
   * @return The created checkbox (owned by the panel)
   */
  QCheckBox* addToggle(const QString& label, bool initially_enabled);

 private Q_SLOTS:
  /**
   * @brief Apply the toggled state of the sending checkbox to its display.
   * @param checked New checkbox state
   */
  void onToggled(bool checked);

 private:
  QVBoxLayout* layout_;  ///< vertical column of checkboxes
  std::map<QObject*, rviz::Display*>
      checkbox_to_display_;  ///< maps each checkbox to the display it toggles
};

#endif  // FLATLAND_VIZ_DIAGNOSTIC_LAYER_PANEL_H
