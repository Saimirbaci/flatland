/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2026 Avidbots Corp.
 * @name   fault_injection_test.cpp
 * @brief  gtest unit tests for the fault ramp/trigger/registry core
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

#include <flatland_plugins/fault_injection_registry.h>
#include <gtest/gtest.h>

using namespace flatland_plugins;

namespace {
SeverityProfile MakeProfile(RampProfile shape, double onset, double up,
                            double hold, double down, double peak) {
  SeverityProfile p;
  p.profile = shape;
  p.onset_time = onset;
  p.ramp_up = up;
  p.hold = hold;
  p.ramp_down = down;
  p.peak_severity = peak;
  return p;
}
}  // namespace

// --- SeverityAt: linear envelope ------------------------------------------
TEST(SeverityAtTest, LinearRampUpHoldRampDown) {
  // onset 2, ramp_up 2, hold 4, ramp_down 2, peak 1.0
  SeverityProfile p = MakeProfile(RampProfile::kLinear, 2, 2, 4, 2, 1.0);

  EXPECT_NEAR(SeverityAt(p, 0.0), 0.0, 1e-9);   // before onset
  EXPECT_NEAR(SeverityAt(p, 2.0), 0.0, 1e-9);   // at onset
  EXPECT_NEAR(SeverityAt(p, 3.0), 0.5, 1e-9);   // mid ramp-up
  EXPECT_NEAR(SeverityAt(p, 4.0), 1.0, 1e-9);   // peak reached
  EXPECT_NEAR(SeverityAt(p, 6.0), 1.0, 1e-9);   // hold
  EXPECT_NEAR(SeverityAt(p, 8.0), 1.0, 1e-9);   // end of hold
  EXPECT_NEAR(SeverityAt(p, 9.0), 0.5, 1e-9);   // mid ramp-down
  EXPECT_NEAR(SeverityAt(p, 10.0), 0.0, 1e-9);  // fully ramped down
  EXPECT_NEAR(SeverityAt(p, 20.0), 0.0, 1e-9);  // after window
}

TEST(SeverityAtTest, PeakScalesMagnitude) {
  SeverityProfile p = MakeProfile(RampProfile::kLinear, 0, 2, 2, 0, 0.4);
  EXPECT_NEAR(SeverityAt(p, 1.0), 0.2, 1e-9);  // half of peak 0.4
  EXPECT_NEAR(SeverityAt(p, 2.0), 0.4, 1e-9);
}

// --- SeverityAt: step envelope --------------------------------------------
TEST(SeverityAtTest, StepIsFlatAcrossWindow) {
  SeverityProfile p = MakeProfile(RampProfile::kStep, 1, 1, 2, 1, 0.8);
  EXPECT_NEAR(SeverityAt(p, 0.5), 0.0, 1e-9);  // before onset
  EXPECT_NEAR(SeverityAt(p, 1.0), 0.8, 1e-9);  // jumps to peak at onset
  EXPECT_NEAR(SeverityAt(p, 3.0), 0.8, 1e-9);  // still peak inside window
  EXPECT_NEAR(SeverityAt(p, 5.1), 0.0, 1e-9);  // after window (1+1+2+1=5)
}

// --- SeverityAt: exponential envelope -------------------------------------
TEST(SeverityAtTest, ExpRampIsMonotonicAndBounded) {
  SeverityProfile p = MakeProfile(RampProfile::kExp, 0, 4, 0, 0, 1.0);
  double prev = -1.0;
  for (double t = 0.0; t <= 4.0; t += 0.5) {
    double s = SeverityAt(p, t);
    EXPECT_GE(s, 0.0);
    EXPECT_LE(s, 1.0 + 1e-9);
    EXPECT_GE(s, prev - 1e-9);  // non-decreasing during ramp-up
    prev = s;
  }
  EXPECT_NEAR(SeverityAt(p, 0.0), 0.0, 1e-9);
  EXPECT_NEAR(SeverityAt(p, 4.0), 1.0, 1e-9);
}

// --- SeverityAt: indefinite hold ------------------------------------------
TEST(SeverityAtTest, IndefiniteHoldNeverEnds) {
  SeverityProfile p = MakeProfile(RampProfile::kLinear, 1, 1, -1, 2, 1.0);
  EXPECT_NEAR(SeverityAt(p, 2.0), 1.0, 1e-9);
  EXPECT_NEAR(SeverityAt(p, 1000.0), 1.0, 1e-9);  // hold < 0 -> never decays
}

// --- ConditionMet ----------------------------------------------------------
TEST(ConditionMetTest, ThresholdComparison) {
  EXPECT_FALSE(ConditionMet("distance_travelled", 0.9, 1.0));
  EXPECT_TRUE(ConditionMet("distance_travelled", 1.0, 1.0));
  EXPECT_TRUE(ConditionMet("x_greater", 5.5, 5.0));
}

// --- Parsing ---------------------------------------------------------------
TEST(ParseTest, FaultKindAndProfile) {
  EXPECT_EQ(ParseFaultKind("sensor_bias"), FaultKind::kSensorBias);
  EXPECT_EQ(ParseFaultKind("torque_loss"), FaultKind::kTorqueLoss);
  EXPECT_EQ(ParseFaultKind("laser_sector_occlusion"),
            FaultKind::kLaserSectorOcclusion);
  // Localization / odometry faults.
  EXPECT_EQ(ParseFaultKind("encoder_drift"), FaultKind::kEncoderDrift);
  EXPECT_EQ(ParseFaultKind("odom_slip"), FaultKind::kOdomSlip);
  EXPECT_EQ(ParseFaultKind("amcl_divergence"), FaultKind::kAmclDivergence);
  // Actuator-stage drivetrain faults.
  EXPECT_EQ(ParseFaultKind("motor_degradation"),
            FaultKind::kMotorDegradation);
  EXPECT_EQ(ParseFaultKind("asymmetric_wheel_speed"),
            FaultKind::kAsymmetricWheelSpeed);
  EXPECT_EQ(ParseFaultKind("locked_wheel"), FaultKind::kLockedWheel);
  EXPECT_EQ(ParseFaultKind("controller_latency"),
            FaultKind::kControllerLatency);
  EXPECT_EQ(ParseFaultKind("not_a_fault"), FaultKind::kUnknown);
  EXPECT_EQ(ParseFaultKind("garbage"), FaultKind::kUnknown);

  EXPECT_EQ(ParseRampProfile("step"), RampProfile::kStep);
  EXPECT_EQ(ParseRampProfile("exp"), RampProfile::kExp);
  EXPECT_EQ(ParseRampProfile("anything_else"), RampProfile::kLinear);
}

// --- Registry round-trip + Reset isolation --------------------------------
TEST(RegistryTest, SetGetRoundTripAndReset) {
  auto &reg = FaultInjectionRegistry::Get();
  reg.Reset();

  // Nothing configured -> inactive effect.
  FaultEffect none =
      reg.GetEffect(ComponentKey("robot", "imu"), FaultKind::kSensorBias);
  EXPECT_FALSE(none.active);
  EXPECT_NEAR(none.severity, 0.0, 1e-9);

  std::map<std::string, FaultEffect> snapshot;
  FaultEffect eff;
  eff.active = true;
  eff.severity = 0.7;
  eff.params["bias"] = 0.3;
  snapshot[FaultInjectionRegistry::MakeKey(ComponentKey("robot", "imu"),
                                           FaultKind::kSensorBias)] = eff;
  reg.SetEffects(snapshot);

  FaultEffect got =
      reg.GetEffect(ComponentKey("robot", "imu"), FaultKind::kSensorBias);
  EXPECT_TRUE(got.active);
  EXPECT_NEAR(got.severity, 0.7, 1e-9);
  EXPECT_NEAR(got.Param("bias"), 0.3, 1e-9);
  EXPECT_NEAR(got.Param("missing", -1.0), -1.0, 1e-9);

  // A different kind / component is unaffected.
  EXPECT_FALSE(
      reg.GetEffect(ComponentKey("robot", "imu"), FaultKind::kDropout).active);
  EXPECT_FALSE(
      reg.GetEffect(ComponentKey("robot", "laser"), FaultKind::kSensorBias)
          .active);

  // Reset clears everything.
  reg.Reset();
  EXPECT_FALSE(
      reg.GetEffect(ComponentKey("robot", "imu"), FaultKind::kSensorBias)
          .active);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
