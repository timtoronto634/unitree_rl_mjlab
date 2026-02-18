"""Booster Robotics K1 (22 DOF) constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

K1_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "booster_k1" / "xmls" / "K1_22dof.xml"
)
assert K1_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  # Meshes live in the 'assets/' subdirectory next to the XML.
  update_assets(assets, K1_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(K1_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator config.
#
# Stiffness and damping are derived from the armature values in the MJCF
# using a 2nd-order PD design with:
#   natural_freq = 10 Hz  (10 * 2π rad/s)
#   damping_ratio = 2.0   (critically over-damped)
#   stiffness = armature * natural_freq²
#   damping   = 2 * damping_ratio * armature * natural_freq
##

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10 Hz
DAMPING_RATIO = 2.0

# --- Head (armature=0.002, effort=6 Nm) ---
_ARMATURE_HEAD = 0.002
_STIFFNESS_HEAD = _ARMATURE_HEAD * NATURAL_FREQ**2
_DAMPING_HEAD = 2.0 * DAMPING_RATIO * _ARMATURE_HEAD * NATURAL_FREQ

# --- Arms: shoulder + elbow (armature=0.001, effort=14 Nm) ---
_ARMATURE_ARM = 0.001
_STIFFNESS_ARM = _ARMATURE_ARM * NATURAL_FREQ**2
_DAMPING_ARM = 2.0 * DAMPING_RATIO * _ARMATURE_ARM * NATURAL_FREQ

# --- Hip pitch (armature=0.0478125, effort=30 Nm) ---
_ARMATURE_HIP_PITCH = 0.0478125
_STIFFNESS_HIP_PITCH = _ARMATURE_HIP_PITCH * NATURAL_FREQ**2
_DAMPING_HIP_PITCH = 2.0 * DAMPING_RATIO * _ARMATURE_HIP_PITCH * NATURAL_FREQ

# --- Hip roll (armature=0.0339552, effort=35 Nm) ---
_ARMATURE_HIP_ROLL = 0.0339552
_STIFFNESS_HIP_ROLL = _ARMATURE_HIP_ROLL * NATURAL_FREQ**2
_DAMPING_HIP_ROLL = 2.0 * DAMPING_RATIO * _ARMATURE_HIP_ROLL * NATURAL_FREQ

# --- Hip yaw (armature=0.0282528, effort=20 Nm) ---
_ARMATURE_HIP_YAW = 0.0282528
_STIFFNESS_HIP_YAW = _ARMATURE_HIP_YAW * NATURAL_FREQ**2
_DAMPING_HIP_YAW = 2.0 * DAMPING_RATIO * _ARMATURE_HIP_YAW * NATURAL_FREQ

# --- Knee (armature=0.095625, effort=40 Nm) ---
_ARMATURE_KNEE = 0.095625
_STIFFNESS_KNEE = _ARMATURE_KNEE * NATURAL_FREQ**2
_DAMPING_KNEE = 2.0 * DAMPING_RATIO * _ARMATURE_KNEE * NATURAL_FREQ

# --- Ankle (armature=0.0565, effort=20 Nm) ---
_ARMATURE_ANKLE = 0.0565
_STIFFNESS_ANKLE = _ARMATURE_ANKLE * NATURAL_FREQ**2
_DAMPING_ANKLE = 2.0 * DAMPING_RATIO * _ARMATURE_ANKLE * NATURAL_FREQ

K1_ACTUATOR_HEAD = BuiltinPositionActuatorCfg(
  target_names_expr=("AAHead_yaw", "Head_pitch"),
  stiffness=_STIFFNESS_HEAD,
  damping=_DAMPING_HEAD,
  effort_limit=6.0,
  armature=_ARMATURE_HEAD,
)

K1_ACTUATOR_ARM = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "ALeft_Shoulder_Pitch",
    "Left_Shoulder_Roll",
    "Left_Elbow_Pitch",
    "Left_Elbow_Yaw",
    "ARight_Shoulder_Pitch",
    "Right_Shoulder_Roll",
    "Right_Elbow_Pitch",
    "Right_Elbow_Yaw",
  ),
  stiffness=_STIFFNESS_ARM,
  damping=_DAMPING_ARM,
  effort_limit=14.0,
  armature=_ARMATURE_ARM,
)

K1_ACTUATOR_HIP_PITCH = BuiltinPositionActuatorCfg(
  target_names_expr=("Left_Hip_Pitch", "Right_Hip_Pitch"),
  stiffness=_STIFFNESS_HIP_PITCH,
  damping=_DAMPING_HIP_PITCH,
  effort_limit=30.0,
  armature=_ARMATURE_HIP_PITCH,
)

K1_ACTUATOR_HIP_ROLL = BuiltinPositionActuatorCfg(
  target_names_expr=("Left_Hip_Roll", "Right_Hip_Roll"),
  stiffness=_STIFFNESS_HIP_ROLL,
  damping=_DAMPING_HIP_ROLL,
  effort_limit=35.0,
  armature=_ARMATURE_HIP_ROLL,
)

K1_ACTUATOR_HIP_YAW = BuiltinPositionActuatorCfg(
  target_names_expr=("Left_Hip_Yaw", "Right_Hip_Yaw"),
  stiffness=_STIFFNESS_HIP_YAW,
  damping=_DAMPING_HIP_YAW,
  effort_limit=20.0,
  armature=_ARMATURE_HIP_YAW,
)

K1_ACTUATOR_KNEE = BuiltinPositionActuatorCfg(
  target_names_expr=("Left_Knee_Pitch", "Right_Knee_Pitch"),
  stiffness=_STIFFNESS_KNEE,
  damping=_DAMPING_KNEE,
  effort_limit=40.0,
  armature=_ARMATURE_KNEE,
)

K1_ACTUATOR_ANKLE = BuiltinPositionActuatorCfg(
  target_names_expr=(
    "Left_Ankle_Pitch",
    "Left_Ankle_Roll",
    "Right_Ankle_Pitch",
    "Right_Ankle_Roll",
  ),
  stiffness=_STIFFNESS_ANKLE,
  damping=_DAMPING_ANKLE,
  effort_limit=20.0,
  armature=_ARMATURE_ANKLE,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.75),
  joint_pos={
    "Left_Hip_Pitch": -0.1,
    "Right_Hip_Pitch": -0.1,
    "Left_Knee_Pitch": 0.3,
    "Right_Knee_Pitch": 0.3,
    "Left_Ankle_Pitch": -0.2,
    "Right_Ankle_Pitch": -0.2,
  },
  joint_vel={".*": 0.0},
)

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.68),
  joint_pos={
    "Left_Hip_Pitch": -0.3,
    "Right_Hip_Pitch": -0.3,
    "Left_Knee_Pitch": 0.65,
    "Right_Knee_Pitch": 0.65,
    "Left_Ankle_Pitch": -0.35,
    "Right_Ankle_Pitch": -0.35,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

# All named *_collision geoms. Feet get condim=3 + friction; others condim=1.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  condim={r"^(left|right)_foot_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot_collision$": 1},
  friction={r"^(left|right)_foot_collision$": (0.6,)},
)

FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
  geom_names_expr=(".*_collision",),
  contype=0,
  conaffinity=1,
  condim={r"^(left|right)_foot_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot_collision$": 1},
  friction={r"^(left|right)_foot_collision$": (0.6,)},
)

# Feet-only: only foot box geoms collide with the world.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(r"^(left|right)_foot_collision$",),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

##
# Final config.
##

K1_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    K1_ACTUATOR_HEAD,
    K1_ACTUATOR_ARM,
    K1_ACTUATOR_HIP_PITCH,
    K1_ACTUATOR_HIP_ROLL,
    K1_ACTUATOR_HIP_YAW,
    K1_ACTUATOR_KNEE,
    K1_ACTUATOR_ANKLE,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_k1_robot_cfg() -> EntityCfg:
  """Get a fresh K1 robot configuration instance."""
  return EntityCfg(
    init_state=HOME_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=K1_ARTICULATION,
  )


K1_ACTION_SCALE: dict[str, float] = {}
for _a in K1_ARTICULATION.actuators:
  assert isinstance(_a, BuiltinPositionActuatorCfg)
  _e = _a.effort_limit
  _s = _a.stiffness
  assert _e is not None
  for _n in _a.target_names_expr:
    K1_ACTION_SCALE[_n] = 0.25 * _e / _s


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_k1_robot_cfg())
  viewer.launch(robot.spec.compile())
