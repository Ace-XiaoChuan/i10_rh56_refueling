# i10 + RH56 MoveIt planning model

This package contains a derived planning description and a headless model audit.
It includes a headless MTC approach task and exports geometric trajectories for
MuJoCo kinematic playback. No trajectory execution, ros2_control hardware or
live MuJoCo/ROS state bridge is provided. RViz is not required.

The authoritative scope, numerical audit settings, provenance boundaries and
results are in [the project contract](../../doc/assembly_contract.md#p0-moveit-规划模型建立2026-10-01).

## Build and audit

From the workspace root, use the project virtual environment to regenerate:

```bash
.venv/bin/python tools/build_moveit_model.py
python3 tools/check_moveit_model_sources.py
source /opt/ros/humble/setup.bash
colcon build --packages-select i10_rh56_moveit_config
source install/setup.bash
mkdir -p artifacts/planning_model
ros2 launch i10_rh56_moveit_config audit.launch.py coverage:=source \
  output_path:="$PWD/artifacts/planning_model/source_audit.yaml"
ros2 launch i10_rh56_moveit_config audit.launch.py coverage:=planning \
  output_path:="$PWD/artifacts/planning_model/planning_audit.yaml"
```

Run the source/hash check after changing any scene, input mesh, candidate or IK
report; regenerate and rebuild before loading stale models. Installed package
files are copies and require a rebuild after regeneration. The launch entry checks installed-file hashes and propagates audit failures.
The source check additionally detects changed workspace inputs. Do not interpret
the existence of an old report as a fresh success.

Generator requirements: current project `.venv` (MuJoCo, NumPy, trimesh) and the
already installed system `/usr/bin/python3` with SciPy for convex hulls. Python
interpreters communicate through JSON; their import paths are not mixed.
`~/mujoco-env` has a different MuJoCo version from the saved candidate and is not
used by this generator. Source manifests record exact versions and file hashes.

## Configuration

- `config/robot.urdf`, `robot.srdf`: default planning model, with added sensor,
  adapter and explicitly documented fixed mounting exclusions.
- `config/robot_source_coverage.*`: diagnostic baseline without those additions;
  matches the existing MuJoCo collision coverage, not a complete planning model.
- `config/kinematics.json`: KDL solver for the six-joint `arm` chain ending at
  hand root `base`. JSON is valid YAML and loaded as ROS parameters by launch.
- `config/scene.json`: baseline scene shapes in world coordinates, including
  separately represented tank cavity pieces, cap, open door and ground. This is
  a snapshot, not a live scene synchronizer. The ground is enabled in the
  planning profile and MTC task; the fixed base support pair alone is allowed.
- `config/audit_reference.json`: deterministic MuJoCo FK references with varying
  hand preforms, original IK solutions and collision observations.
- `config/provenance.json`, `generated_hashes.json`: inputs, derived mesh mapping,
  collision exclusions, model limitations and stale-file checks.

All model meshes use package URIs. Runtime loading therefore uses the installed
package, without absolute workspace mesh paths. Regeneration still requires the
source workspace and archived inputs.

The `hand` group has no solver and must remain at the supplied preform in arm
planning. Its zero velocity/effort fields block hand trajectory use; they are
not hardware ratings. Mimic joints are an ideal kinematic approximation of the
source equality constraints, not reconstructed contact-deformed finger states.
No controller, fake execution feedback or acceleration limit is introduced.
The P1 simulation start and approach settings are confirmed in the project contract.

The package manifest does not grant new rights to derived third-party geometry;
original licenses and provenance remain under `assets/`. Installation does not
validate hardware mounting, internal assembly fit or physical grasp stability.

## MTC approach and MuJoCo playback

The scope and numerical settings are maintained in the P1 contract section.
Prepare a request with explicitly confirmed start joints and axial offset:

```bash
.venv/bin/python tools/prepare_cap_approach.py --start 0 -.5 1.2 0 0 0 \
  --distance .1 --output artifacts/planning/approach_request.json
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/mtc_approach"
ros2 launch i10_rh56_moveit_config approach.launch.py \
  request_path:="$PWD/artifacts/planning/approach_request.json" \
  output_path:="$PWD/artifacts/planning/approach.json"
.venv/bin/python tools/replay_cap_approach.py --view
```

`FixedState` loads the exact start with fixed hand preform, `MoveTo` uses OMPL
RRTConnect, and `MoveRelative` uses CartesianPath along the estimated cap axis.
There is no state nudging, controller execution, grasp closure, attach or twist.
The request labels its observation as simulation ground truth. It is not vision.
The exported points have no hardware timing; display duration only changes
playback speed. Output from a failed launch cannot be accepted by the replay.

Playback checks geometry provenance, joint order, fixed hand preform, limits,
continuity, original MuJoCo collisions and endpoint pose. MoveIt additionally
checks its full planning collision coverage. Finite sampling does not prove
continuous collision freedom. Sensor/adapter and ground collision differences
from MuJoCo remain as described in the P0 contract.

Viewer keys: Space pauses, R restarts explicitly, F changes camera, C shows
collision geometry. Playback holds the final pose. Use `--auto-close` for a
single validation run. `--render-dir artifacts/planning/views` saves an APNG
animation and static frames; set `MUJOCO_GL=glfw` on this WSLg environment.


## One new random scene

For the authorized single mixed sample, add `--random-seed SEED` to
`tools/prepare_cap_approach.py`, with explicit `--start`, `--distance` and a
separate output directory. Confirmed settings and the actual seed are maintained
in the project contract's P2 preliminary single-sample section. This draws one
sample before feasibility checks, not the existing multi-group regression set.
The request includes the sample, observation and complete world-shape snapshot;
MTC loads that snapshot instead of the fixed baseline. Meshes and exclusions
remain unchanged. Failures retain the input and logs; do not select a new seed
because planning failed.

After building and sourcing the workspace, the saved request can be planned with:

```bash
ros2 launch i10_rh56_moveit_config approach.launch.py \
  request_path:="$PWD/artifacts/planning/random_single/approach_request.json" \
  output_path:="$PWD/artifacts/planning/random_single/approach.json"
.venv/bin/python tools/replay_cap_approach.py \
  --trajectory artifacts/planning/random_single/approach.json --view
```

Replay reconstructs the sampled MuJoCo world and rejects changed sampling,
world shapes/poses, exclusions, candidate or approach targets. Saved animation:
`artifacts/planning/random_single/views/index.html`. This is one geometric
approach example, not complete P2 regression, dynamics or physical grasp proof.


## Fixed-depth rotation screening

The P3 first-round contract defines numerical settings and limitations. This
screens the current approach endpoint with the hand still in its approach
preform and an assumed fixed cap/hand transform. It does not close the fingers,
model screw motion, perform controller execution or automatically select a new
branch after failure.

```bash
.venv/bin/python tools/check_cap_rotation.py
```

A nonzero exit on a detected limit preserves `diagnostic.json` and the checked
prefix. Stop the diagnostic; do not rerun or replace the sample. The saved prefix
can be audited independently after building and sourcing the workspace:

```bash
ros2 launch i10_rh56_moveit_config rotation_audit.launch.py \
  diagnostic_path:="$PWD/artifacts/planning/rotation_single/diagnostic.json" \
  output_path:="$PWD/artifacts/planning/rotation_single/fcl_validation.json"
.venv/bin/python tools/preview_cap_rotation.py --view
```

Only the common checked prefix is displayed. `summary.json` retains the failure
and unfinished target. FCL audits robot/world and self collisions with the cap
world objects updated at each angle; it does not validate world/world screw
contacts. A different IK branch requires its own approach path and rotation
assessment before it can replace the current endpoint. No closed-hand grasp
stability or actual cap removal has been demonstrated.

## Segmented rotation geometry experiment

Confirmed segment settings, actual results and limitations are maintained in the
project contract's segmented rotation sections. Run one experiment in a separate
output directory; existing results are never overwritten or automatically resumed:

```bash
.venv/bin/python tools/check_segmented_rotation.py
MUJOCO_GL=glfw .venv/bin/python tools/preview_segmented_rotation.py \
  --render-dir artifacts/planning/segmented_single/views --view
```

Rotation uses continuous IK and two collision coverage checks. Regrasp geometry
uses MTC axial retreat, MoveTo adjustment and axial approach, followed by MuJoCo
validation. The unchanged hand/cap candidate is recomputed from a fresh estimator
snapshot. Static snapshot verification after retreat is not an online perception
and execution loop. The hand remains in its approach preform; release, closure,
physical support, screw motion and detachment are not executed.

The saved prefix excludes rejected paths. A failed endpoint remains invalid even
if its dense FCL check passed. Future endpoint failures retain errors and rejected
points in a separate non-replayable report mode; this does not reconstruct missing
historical states. The special `--resume-validation` option only repairs the
recorded tuple/JSON-list comparison false alarm using the original hashed path;
it does not resume a failed planning or physical stage.

The authorized tolerance-adjusted run is kept separately. Numerical values and
results are in the contract; requests record `endpoint_tolerances`, with historical
requests retaining their original thresholds. Use an unused directory for a new
explicitly authorized run. Saved-prefix playback for the second run:

```bash
MUJOCO_GL=glfw .venv/bin/python tools/preview_segmented_rotation.py \
  --summary artifacts/planning/segmented_single_tol_020/summary.json --view
```

## Selected regrasp configuration

`selected_regrasp.launch.py` uses the independent `plan_selected_regrasp` executable.
The MTC stages retain the confirmed initial state, retreat, move to the selected
pregrasp **joint** target, and approach along the cap axis. The exported endpoint
must satisfy both pose checks and the selected raw joint configuration, including
its winding. The Python selection tool audits the actual endpoint again before
accepting increased rotation space. This does not execute release, regrasp or
contact motion. Existing approach/rotation tools and source bindings are retained.

Build this package before running `tools/select_regrasp_configuration.py` from
the workspace root. Settings and results are maintained in the [contract](../../doc/assembly_contract.md#任务点-2-第三轮重新抓握构型筛选与路径衔接).

## Selected initial approach configuration

`selected_approach.launch.py` runs the independent `plan_selected_approach`
executable. It keeps the original complete initial state and plans MoveTo to a
selected pregrasp joint target, followed by the original axial MoveRelative.
Both the actual pregrasp and grasp endpoints are checked against raw joint
values, including winding. Dense FCL rejection records identify the stage,
joints, bounds and contact pairs; rejected paths cannot be replayed as successes.

Run `tools/select_initial_configuration.py` from the workspace root after
building this package. The Python tool verifies the actual two-stage path in
MuJoCo and performs a new rotation scan and full FCL audit from that actual
endpoint. `--verify-only` checks saved sources and geometry without replanning.
The [contract](../../doc/assembly_contract.md#任务点-2-第四轮初始到达构型筛选)
maintains budgets, outcomes and verification limits. The existing approach and
regrasp executables remain unchanged; no model regeneration or hardware control
is involved.
