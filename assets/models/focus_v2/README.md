# focus_v2 — Joysway Focus V2 (1 m RC sailboat)

A wind-driven sailboat for LOTUSim. Unlike the powered vessels in the library, it
carries **no thruster**: thrust comes from the wind on a **sail**, modelled as an
xdyn **three-foil rig**.

Everything here is re-authored from published dimensions of the Joysway Focus V2.
The ecosystem is EPL-2.0: manufacturer CAD and third-party meshes cannot be vendored,
whatever their apparent convenience, so the hull and rig were measured and rebuilt
rather than imported.

## How it is modelled

| Element | xdyn force model | Command |
|---|---|---|
| Sail | `aerodynamic polar` (apparent wind, `air rho`) | `mainsail(sheet)` — trim angle |
| Keel | `hydrodynamic polar`, fixed | — (resists leeway) |
| Rudder | `hydrodynamic polar`, steered | `rudder(helm)` — helm angle |
| Hull | mesh hydrostatics + `non-linear hydrostatic (fast)` + quadratic/linear damping | — |
| Righting | low centre of inertia (lead keel bulb) | — |

The commands are the angle-command signals seeded by the world's
`<control_surfaces>` block. Frames follow xdyn (NED; body X-fwd, Y-stbd, Z-down):
the STL is authored in body axes (bow +X), the gz visual OBJ bow on +Y.

## ⚠️ Physics is estimated, not validated

Geometry (LOA 0.995 m, beam 0.17 m, mass 2.93 kg, sail area ~0.555 m², keel
~0.39 m) is from published Focus V2 specs. **Every hydro/aero coefficient, inertia,
added-mass and damping value is an engineering estimate** (each marked
`# ESTIMATED — pending naval-engineer validation`), tuned only to *sail plausibly*.
This is **not** tank-validated hydrodynamics.

## ⚠️ Numerical stability — use a fine time step

This model is **numerically stiff**. With a **fixed-step rk4 at dt = 0.02 s** it **diverges**
(vertical position → ∞ → NaN, and the boat appears to spin 360°). This is a **solver artifact,
not a physics problem**: at **dt = 0.005 s** (or with the adaptive `rkck` solver in standalone
mode) the boat sails cleanly — no divergence, no uncommanded 360°. Three independent stable
configurations (rkck@0.02, rk4@0.005, rk4@0.001) agree on the same ~1 m/s reach speed.

Consequences:
- **Standalone / prediction mode:** run with `-s rkck` (adaptive) or `-s rk4 --dt 0.005`.
- **Co-simulation:** the websocket step server (`xdyn-for-cs`) requires a monotonic clock, so
  `rkck` is **not** usable there (it back-tracks in time). Drive it with **`-s rk4` and a step
  `Dt ≤ 0.005`** (the client controls `Dt` per message), or de-stiffen the model. The gz plugin's
  default `dt = 0.02` is the value that diverges.

Historical note: the June 2026 "the boat does 360° / diverges" blocker was this stiffness at
dt = 0.02, *not* broken hydrodynamics. The yaw damping here is modest (directional stability), not
a crutch.

## Verified points of sail (closed loop)

Measured with a thin Python ↔ `xdyn-for-cs` autopilot (holds a heading, no gz/ROS): honest
no-go zone (the boat makes **negative** headway pointing into the wind), speed rising toward the
beam reach, moderate downwind. Sail trim (`mainsail(sheet)`) and steering (`rudder(helm)`) are
both live, meaningful inputs.

## Run the demo

Needs the quaternion and `<control_surfaces>` engine fixes. Start an `xdyn-for-cs`
server on the model and Gazebo on `assets/worlds/focus_v2_demo.world`, then drive it
with the closed-loop helmsman (`focus_v2` package in LOTUSim-generic-scenario).

**Real-behaviour proof:** headless xdyn + Gazebo co-simulation, the boat sails up to
the race mark and rounds it — **closest approach 0.20 m**, sweeping ~108° around the
buoy, then sails on. No divergence.
