/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   diagnostic_layer_panel.cpp
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

#include "flatland_viz/diagnostic_layer_panel.h"

#include <QCheckBox>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>

#include "rviz/display.h"

DiagnosticLayerPanel::DiagnosticLayerPanel(QWidget* parent)
    : QDockWidget("Diagnostic Layers", parent) {
  setObjectName("DiagnosticLayers");

  QWidget* container = new QWidget(this);
  layout_ = new QVBoxLayout(container);
  layout_->setContentsMargins(6, 6, 6, 6);
  layout_->setAlignment(Qt::AlignTop);
  container->setLayout(layout_);

  // Scroll in case more overlays are registered than fit the dock.
  QScrollArea* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setWidget(container);
  setWidget(scroll);
}

void DiagnosticLayerPanel::addLayer(const QString& label,
                                    rviz::Display* display,
                                    bool initially_enabled) {
  QCheckBox* checkbox = new QCheckBox(label, this);
  checkbox->setChecked(initially_enabled);

  if (display != nullptr) {
    display->setEnabled(initially_enabled);
    checkbox_to_display_[checkbox] = display;
    connect(checkbox, SIGNAL(toggled(bool)), this, SLOT(onToggled(bool)));
  } else {
    // No backing display (e.g. the overlay's publisher is absent); show the row
    // but make it inert so the user sees the layer exists.
    checkbox->setEnabled(false);
  }

  layout_->addWidget(checkbox);
}

QCheckBox* DiagnosticLayerPanel::addToggle(const QString& label,
                                           bool initially_enabled) {
  QCheckBox* checkbox = new QCheckBox(label, this);
  checkbox->setChecked(initially_enabled);
  layout_->addWidget(checkbox);
  return checkbox;
}

void DiagnosticLayerPanel::onToggled(bool checked) {
  auto it = checkbox_to_display_.find(sender());
  if (it != checkbox_to_display_.end() && it->second != nullptr) {
    it->second->setEnabled(checked);
  }
}
