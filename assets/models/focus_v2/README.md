# focus_v2 — Joysway Focus V2 (1 m RC sailboat)

A wind-driven sailboat for LOTUSim. Unlike the powered vessels in the library, it
carries **no thruster**: thrust comes from the wind on a **sail**, modelled as an
xdyn **three-foil rig**.

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

## Run the demo

Needs the quaternion and `<control_surfaces>` engine fixes. Start an `xdyn-for-cs`
server on the model and Gazebo on `assets/worlds/focus_v2_demo.world`, then drive it
with the closed-loop helmsman (`focus_v2` package in LOTUSim-generic-scenario).

**Real-behaviour proof:** headless xdyn + Gazebo co-simulation, the boat sails up to
the race mark and rounds it — **closest approach 0.20 m**, sweeping ~108° around the
buoy, then sails on. No divergence.
