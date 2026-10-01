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
