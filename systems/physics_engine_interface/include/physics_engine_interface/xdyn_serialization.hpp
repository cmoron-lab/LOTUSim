/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef PHYSICS_ENGINE_INTERFACE__XDYN_SERIALIZATION_HPP_
#define PHYSICS_ENGINE_INTERFACE__XDYN_SERIALIZATION_HPP_

#include <gz/math/Quaternion.hh>

namespace lotusim::gazebo {

// xdyn transmits an orientation as four named scalar fields. The ordering is
// real-part-first: (qr, qi, qj, qk) maps to gz::math::Quaterniond(w, x, y, z).
// Packing (send) and unpacking (receive) MUST be exact inverses, otherwise a
// pure yaw on the wire comes back as a different axis (the j/k fields were
// historically swapped on receive, aliasing yaw onto roll, so a steered vessel
// never accumulated heading and dead-reckoned in a straight line).
struct XdynQuatFields
{
    double qr;  // real / scalar  (w)
    double qi;  // i              (x)
    double qj;  // j              (y)
    double qk;  // k              (z)
};

// Pack a quaternion into xdyn's named fields (used when sending state to xdyn).
inline XdynQuatFields quatToXdynFields(const gz::math::Quaterniond& q)
{
    return XdynQuatFields{q.W(), q.X(), q.Y(), q.Z()};
}

// Rebuild a quaternion from xdyn's named fields (used when receiving state).
// gz::math::Quaterniond's constructor takes (w, x, y, z) == (qr, qi, qj, qk).
inline gz::math::Quaterniond
quatFromXdynFields(double qr, double qi, double qj, double qk)
{
    return gz::math::Quaterniond(qr, qi, qj, qk);
}

}  // namespace lotusim::gazebo

#endif  // PHYSICS_ENGINE_INTERFACE__XDYN_SERIALIZATION_HPP_
