/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/xdyn_serialization.hpp"

#include <gtest/gtest.h>
#include <gz/math/Quaternion.hh>

#include <cmath>

using gz::math::Quaterniond;
using lotusim::gazebo::quatFromXdynFields;
using lotusim::gazebo::quatToXdynFields;

namespace {
void expectQuatNear(
    const Quaterniond& a,
    const Quaterniond& b,
    double tol = 1e-12)
{
    EXPECT_NEAR(a.W(), b.W(), tol);
    EXPECT_NEAR(a.X(), b.X(), tol);
    EXPECT_NEAR(a.Y(), b.Y(), tol);
    EXPECT_NEAR(a.Z(), b.Z(), tol);
}
}  // namespace

// Packing then unpacking must reproduce the original quaternion exactly.
TEST(XdynSerialization, PackUnpackRoundTripIsIdentity)
{
    const double d2r = M_PI / 180.0;
    for (double roll : {0.0, 12.0, -30.0}) {
        for (double pitch : {0.0, 8.0, -15.0}) {
            for (double yaw : {0.0, 30.0, 90.0, 175.0, -120.0}) {
                Quaterniond q(roll * d2r, pitch * d2r, yaw * d2r);
                auto f = quatToXdynFields(q);
                expectQuatNear(q, quatFromXdynFields(f.qr, f.qi, f.qj, f.qk));
            }
        }
    }
}

// A pure heading change must survive the round-trip as a pure heading change.
TEST(XdynSerialization, PureYawSurvivesRoundTrip)
{
    const double yaw = 30.0 * M_PI / 180.0;
    Quaterniond q(0.0, 0.0, yaw);
    auto f = quatToXdynFields(q);
    auto out = quatFromXdynFields(f.qr, f.qi, f.qj, f.qk);
    auto e = out.Euler();  // (roll, pitch, yaw)
    EXPECT_NEAR(e.X(), 0.0, 1e-9);
    EXPECT_NEAR(e.Y(), 0.0, 1e-9);
    EXPECT_NEAR(e.Z(), yaw, 1e-9);
}

// Regression guard: the historical bug fed the k field where j was expected
// (and vice-versa). On a pure yaw this corrupts the heading entirely.
TEST(XdynSerialization, SwappedJKCorruptsHeading)
{
    const double yaw = 30.0 * M_PI / 180.0;
    Quaterniond q(0.0, 0.0, yaw);
    auto f = quatToXdynFields(q);
    Quaterniond buggy = quatFromXdynFields(f.qr, f.qi, f.qk, f.qj);  // j/k swap
    auto e = buggy.Euler();
    EXPECT_NEAR(e.Z(), 0.0, 1e-6);                       // heading is lost
    EXPECT_GT(std::abs(e.X()) + std::abs(e.Y()), 1e-3);  // and leaks elsewhere
}
