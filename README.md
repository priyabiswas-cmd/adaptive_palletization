# UR10e Palletizing (MATLAB + Robotics System Toolbox + Simulink 3D Animation)

A UR10e on a raised platform with a Robotiq EPick 4-cup vacuum gripper picks boxes from a
static conveyor and places them on a Euro pallet. The plan comes from `config/boxes.json`.
RRT handles the large moves and a state machine handles the vertical 5 cm approach moves.

## Conventions
| Item | Convention |
|---|---|
| Units | meters, kilograms; radians inside the code |
| World frame | Origin on the floor directly under the robot base, z up. **The JSON and the simulation use the same frame.** |
| Box position | `x, y` = centre of the bottom face, `z` = height of the bottom face |
| Gripper | EPick 4-cup vacuum, cups facing straight down. TCP (`suction_tcp`) = centre of the cup faces, 0.19 m from `tool0` |
| Grasp point | Centre of the box top face: `z + h` |
| Approach height | 5 cm above the top face: `z + h + 0.05` (pre-pick, post-pick, pre-place, post-place) |
| Robot | UR10e, base mounted at z = 0.51 on a 0.25 x 0.25 x 0.5 m platform at the origin |
| Conveyor | **Static** (belt does not move), top surface at z = 0.50, along x at y = 0.70 |
| Box spawning | Like the MathWorks example: box 1 is on the conveyor at the start; each next box appears at its JSON `start` when the previous box has been placed. All boxes may share one start; their `goal`s (pallet pattern) must not overlap. |
| Pallet | Euro pallet 1.2 x 0.8 m, top surface at z = 0.144 |

## Folder layout
| Folder | Contents |
|---|---|
| `config/` | `boxes.json`: the task plan, in pick order |
| `utils/` | `loadTaskPlan` (reads and checks the JSON), terminal logger |
| `robot/` | `setupRobot` (UR10e + platform + gripper rigidBodyTree), `solveTcpIK` (closed-form UR IK), `robotConfig` |
| `environment/` | Work cell for the 3D scene: `buildWorkCell` (floor, platform, static conveyor, Euro pallet, boxes from the JSON), `setBoxPosition`, `getCollisionObjects`. `toSim3d` is the only place where our frame is converted to the engine's frame. |
| `simulink/` | `buildPalletizingModel` (creates `palletizing.slx`), `PalletizingController` (the "Task Controller" block that outputs joint angles) |
| `palletizing.slx` | The Simulink model: same simulator, blocks and camera as the MathWorks example |
| `setupPaths.m` | Adds all project folders to the MATLAB path |
| `rrt/` | `planJointPath` (manipulatorRRT with collision checking, held box attached under the cups), `timeTrajectory` (contopptraj timing, as in the example), `checkTrajectory` (sample-by-sample collision check), `rrtConfig` (settings) |
| `state_machine/` | `PalletizingStateMachine` (all states and transitions, terminal messages), `verticalTrajectory` (straight 5 cm up/down moves), `stateMachineConfig` (approach height, speeds, suction time) |
| `main.m` | Runs everything |

## Task plan format (`config/boxes.json`)
```json
{ "id": 1, "l": 0.20, "b": 0.20, "h": 0.20, "weight": 1.0,
  "start": { "x": -0.60, "y": 0.70, "z": 0.50 },
  "goal":  { "x": -0.60, "y": -0.25, "z": 0.144 } }
```
The `goal` values are the pallet pattern. A pallet-pattern optimizer only needs to write these
(and the order of the boxes); nothing else in the code has to change.
`loadTaskPlan()` returns a struct array with fields `id, l, b, h, weight, start [x y z], goal [x y z]`.
It raises an error for missing fields, non-positive values, duplicate ids, or overlapping boxes.

## Robot interface
```matlab
[tree, cfg] = setupRobot();                       % rigidBodyTree + parameters
q = solveTcpIK(tree, cfg, [x y z], qCurrent);     % TCP position (cups down) -> joints, [] if unreachable
T = getTransform(tree, q, cfg.tcpBodyName);       % joints -> TCP pose
```

## Running the simulation
```matlab
setupPaths                  % once per MATLAB session
buildPalletizingModel       % only needed to (re)create palletizing.slx
open_system('palletizing')  % then press Run; the 3D viewer opens with the example's camera
```

## Simulink model (`palletizing.slx`)
| Block | What it does |
|---|---|
| Simulation 3D Scene Configuration | Unreal "Empty scene", camera exactly as in the MathWorks example |
| Work Cell (Simulation 3D Actor) | Runs `buildWorkCell(World, Actor)` at start |
| Task Controller (MATLAB System) | `PalletizingController`: outputs the 6 joint angles every 0.02 s |
| Robot (Simulation 3D Robot) | UR10e + EPick from `setupRobot()`, driven by the Task Controller |

## State machine
For every box in the JSON (in order):

| State | Who moves the robot | What happens |
|---|---|---|
| `MOVE_TO_PREPICK` | RRT ON | current pose -> 5 cm above the box top on the conveyor |
| `DESCEND_TO_PICK` | state machine | straight down to the box top (`start.z + h`) |
| `GRASP` | state machine | suction on, box attached (0.5 s) |
| `ASCEND_TO_POSTPICK` | state machine | straight up 5 cm with the box |
| `MOVE_TO_PREPLACE` | RRT ON | carry the box to 5 cm above its goal (box included in collision checks) |
| `DESCEND_TO_PLACE` | state machine | straight down until the box rests on its goal (`goal.z + h`) |
| `RELEASE` | state machine | suction off, box left exactly at the goal; next box appears on the conveyor |
| `ASCEND_TO_POSTPLACE` | state machine | straight up 5 cm |

After the last box: `MOVE_TO_HOME` (RRT) -> `DONE`. If a motion cannot be planned: `FAILED` (robot stops).
Every transition is printed in the terminal, e.g.
`[  10.14 s] [STATE]  RELEASE  box 1: suction OFF, box placed at [-0.600 -0.250 0.144]`.

The state machine does not depend on Simulink; it can be run from the command line:
```matlab
env.spawnBox = @(k) [];  env.moveBox = @(k, p) [];      % no 3D world
sm = PalletizingStateMachine(tree, cfg, loadTaskPlan(), env);
while ~sm.isDone(), q = sm.step(); end
```

## RRT interface
```matlab
obs   = getCollisionObjects(boxes, boxPositions, heldBoxIndex, rrtConfig().obstacleMargin);
wp    = planJointPath(tree, cfg, qStart, qGoal, obs, heldBox);   % heldBox = [] when empty-handed
qTraj = timeTrajectory(wp);                                      % one row per 0.02 s step
```

## Environment interface
```matlab
world = sim3d.World.getWorld('palletizing');       % during a simulation
spawnBox(world, box);                              % make a box appear at its JSON start
setBoxPosition(world, box, bottomCentre);          % move a box (bottom-face centre, world frame)
objs = getCollisionObjects(boxes, boxPositions, heldBox); % collisionBox list for the planner
```
