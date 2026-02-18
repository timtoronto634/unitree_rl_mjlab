"""Booster K1 velocity environment configurations."""

from mjlab.asset_zoo.robots import (
  K1_ACTION_SCALE,
  get_k1_robot_cfg,
)
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg


def booster_k1_rough_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create Booster K1 rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  cfg.scene.entities = {"robot": get_k1_robot_cfg()}

  site_names = ("left_foot", "right_foot")

  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(
      mode="subtree",
      pattern=r"^(left_foot_link|right_foot_link)$",
      entity="robot",
    ),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  self_collision_cfg = ContactSensorCfg(
    name="self_collision",
    primary=ContactMatch(mode="subtree", pattern="Trunk", entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern="Trunk", entity="robot"),
    fields=("found",),
    reduce="none",
    num_slots=1,
  )
  cfg.scene.sensors = (feet_ground_cfg, self_collision_cfg)

  # K1 has more collision geoms than G1; raise nconmax to avoid overflow.
  cfg.sim.nconmax = 100

  if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
    cfg.scene.terrain.terrain_generator.curriculum = True

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = K1_ACTION_SCALE

  cfg.viewer.body_name = "Trunk"

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.viz.z_offset = 1.1

  cfg.events["base_com"].params["asset_cfg"].body_names = ("Trunk",)

  # Rationale for std values:
  # - Hip pitch / knee get the loosest std for natural leg bending during stride.
  # - Hip roll / yaw stay tight to prevent lateral sway.
  # - Ankle roll is very tight for balance; ankle pitch looser for foot clearance.
  # - K1 has no waist joints.
  # - Shoulder / elbow get moderate freedom for natural arm swing.
  # - Head joints are kept very tight since they don't affect locomotion.
  cfg.rewards["pose"].params["std_standing"] = {".*": 0.05}
  cfg.rewards["pose"].params["std_walking"] = {
    # Lower body.
    r".*_Hip_Pitch$": 0.5,
    r".*_Hip_Roll$": 0.15,
    r".*_Hip_Yaw$": 0.15,
    r".*_Knee_Pitch$": 0.5,
    r".*_Ankle_Pitch$": 0.15,
    r".*_Ankle_Roll$": 0.1,
    # Arms.
    r".*Shoulder_Pitch$": 0.15,
    r".*_Shoulder_Roll$": 0.1,
    r".*_Elbow_Pitch$": 0.1,
    r".*_Elbow_Yaw$": 0.1,
    # Head (kept near zero during locomotion).
    r"(AAHead_yaw|Head_pitch)": 0.05,
  }
  cfg.rewards["pose"].params["std_running"] = {
    # Lower body.
    r".*_Hip_Pitch$": 0.5,
    r".*_Hip_Roll$": 0.25,
    r".*_Hip_Yaw$": 0.25,
    r".*_Knee_Pitch$": 0.5,
    r".*_Ankle_Pitch$": 0.25,
    r".*_Ankle_Roll$": 0.1,
    # Arms.
    r".*Shoulder_Pitch$": 0.25,
    r".*_Shoulder_Roll$": 0.1,
    r".*_Elbow_Pitch$": 0.1,
    r".*_Elbow_Yaw$": 0.1,
    # Head.
    r"(AAHead_yaw|Head_pitch)": 0.05,
  }
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("Trunk",)
  cfg.rewards["foot_clearance"].params["asset_cfg"].site_names = site_names
  cfg.rewards["foot_slip"].params["asset_cfg"].site_names = site_names
  cfg.rewards["self_collisions"] = RewardTermCfg(
    func=mdp.self_collision_cost,
    weight=-1.0,
    params={"sensor_name": self_collision_cfg.name},
  )

  # Apply play mode overrides.
  if play:
    cfg.episode_length_s = int(1e9)

    cfg.observations["policy"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.events["randomize_terrain"] = EventTermCfg(
      func=envs_mdp.randomize_terrain,
      mode="reset",
      params={},
    )

    if cfg.scene.terrain is not None:
      if cfg.scene.terrain.terrain_generator is not None:
        cfg.scene.terrain.terrain_generator.curriculum = False
        cfg.scene.terrain.terrain_generator.num_cols = 5
        cfg.scene.terrain.terrain_generator.num_rows = 5
        cfg.scene.terrain.terrain_generator.border_width = 10.0

  return cfg


def booster_k1_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create Booster K1 flat terrain velocity configuration."""
  cfg = booster_k1_rough_env_cfg(play=play)

  # Switch to flat terrain.
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  # Disable terrain curriculum.
  assert "terrain_levels" in cfg.curriculum
  del cfg.curriculum["terrain_levels"]

  return cfg
