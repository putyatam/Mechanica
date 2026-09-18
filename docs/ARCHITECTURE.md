# Mechanica architecture 0.1

## Core rule

The player builds **physics**, not devices.

There is no authoritative `HydraulicCylinder`, `Pump`, `PneumaticActuator`,
`Differential`, or `Engine` gameplay object that creates a result by script.

High-level named assemblies may exist later as blueprints/examples, but they must
compile to the same low-level physical primitives available to the player.

## Simulation layers

1. **CAD document**
   - parametric solids
   - material definitions
   - mates / constraints
   - surfaces and seals
   - editable dimensions

2. **Solid mechanics**
   - Jolt rigid bodies
   - contacts
   - friction
   - joints
   - sleeping/islands
   - rigidly connected CAD parts compiled to compound bodies

3. **Continuum**
   - matter occupies void space
   - mass is conserved
   - pressure follows the medium equation of state
   - openings create flow
   - moving boundaries change volume
   - pressure applies force by integrating over wetted surfaces

4. **Topology compiler**
   - sparse/adaptive voxelization or cut cells
   - detect sealed void regions
   - detect openings/throats between regions
   - maintain local connectivity when mechanisms move
   - generate control volumes + portals automatically

5. **Optional local high-resolution solver**
   - only for regions where lumped control-volume physics is insufficient
   - never run full CFD over the whole machine by default

## Why this is still "from first principles"

A player-made compressor can consist of:
- housing solids
- a rotating eccentric/crank
- a moving plate or piston-like solid
- seals
- mechanically constructed check valves
- inlet/outlet void geometry

The engine does not know it is a compressor.
It only sees changing fluid volumes, openings, contact surfaces and solid motion.

## Near-term roadmap

0.1: generic pressure/flow kernel + Jolt benchmark (current)
0.2: 3D CAD viewport, transform gizmos, primitives, materials
0.3: constraints + assembly compiler + compound bodies
0.4: automatic sealed-volume detection
0.5: moving-boundary pressure coupling to Jolt bodies
0.6: geometry-derived leaks, pipes, throttling and mechanically built valves
0.7: liquid compressibility, cavitation and thermal coupling
0.8: stress/load instrumentation and breakable physical parts


## Version 0.2: game-first build loop

The normal player view is now a full 3D playground with a small hotbar rather than a
multi-panel engineering application.

The editor stores lightweight `BuildPart` records. They do not become rigid bodies while
the player is building.

When Play is pressed:
1. touching parts are grouped into connected components;
2. each connected component is compiled to one Jolt body;
3. multi-part rigid groups use a Jolt static compound shape;
4. disconnected constructions remain separate bodies.

This is deliberately the first step toward a real assembly compiler. When joints arrive,
they will define where a rigid component must be split and how the resulting bodies can
move relative to one another.


## 0.3 — Definition / Instance split

A reusable block is `BlockDefinition`.
A placed object is `BlockInstance`.

`BlockDefinition` contains geometry components and materials. `BlockInstance` contains
world transform, explicit attachment edges and an optional local definition override.

This separation is required for:
- reusable user blocks;
- arbitrary library groups;
- editing an already placed block without destroying the library template;
- instancing the render geometry of unmodified definitions;
- compiling multiple attached instances into one rigid body.

Touching and attachment are deliberately different concepts.

### Shape authoring

The first authoring kernel supports:
- analytic box/cylinder/sphere;
- hollow tube;
- arbitrary planar profile + extrusion;
- profile + revolution;
- composition by copying components from another definition.

The long-term geometry compiler is intentionally independent from gameplay semantics.
Boolean CSG and convex decomposition are the next geometry layer, not new gameplay block types.


## 0.4 — editor, CSG, gizmos and structural links

### Transform tools

The world editor now has a separate manipulation layer based on selection and a group
pivot. The tool itself never changes gameplay semantics.

- Move: applies translation delta to every selected instance.
- Rotate: rotates instance positions around the selection pivot and composes Euler
  rotation with the same delta.
- Scale: scales relative positions around the group pivot and multiplies each instance
  scale.

ImGuizmo is used only as an interaction widget. The world model stays engine-owned.

### Placement attachment mode

Contact candidates are derived from geometry.
Attachments remain explicit graph edges.

Holding Alt freezes the candidate transform and turns the mouse into an attachment
selection cursor. Releasing Alt restores the saved cursor coordinate and returns to
placement.

This keeps placement motion and attachment authoring as two separate user actions.

### CSG

Boolean operations are evaluated as occupancy over a configurable 3D grid. The resulting
occupied cells are greedily merged into large axis-aligned boxes.

Advantages for this project:
- robust holes and concavity;
- same representation can be used by rendering and physics;
- no dynamic triangle mesh dependency;
- bounded complexity;
- deterministic output;
- easy future parallelization / GPU compute.

The editor continues to render analytic primitives when a definition does not need CSG,
so normal cylinders/spheres keep smooth visual geometry.

### Structural material layer

Explicit rigid attachment edges are compiled as FixedConstraints instead of permanently
merging all connected BlockInstances into one body.

The Jolt constraint lambda from the last physics step is interpreted as impulse.
Linear force and angular moment are estimated by dividing by dt. Contact-area stress,
elastic strain and damage are then derived from the physical material properties.

This enables breakable mechanisms without an arbitrary health bar.

The current implementation is a structural-link model, not volumetric FEM. A future
cluster compiler should merge cold/low-stress regions for scale while preserving
measurable links around joints and loaded interfaces.
