"""Sim2Sim deployment script for MuJoCo."""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import onnxruntime as ort
import torch
import yaml
import time


def get_project_root() -> Path:
    """Get the project root directory."""
    return Path(__file__).resolve().parents[2]


def get_gravity_orientation(quaternion: np.ndarray) -> np.ndarray:
    """Compute gravity orientation from quaternion [qw, qx, qy, qz]."""
    qw, qx, qy, qz = quaternion
    gravity_orientation = np.zeros(3)
    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)
    return gravity_orientation


def pd_control(
    target_q: np.ndarray,
    q: np.ndarray,
    kp: np.ndarray,
    target_dq: np.ndarray,
    dq: np.ndarray,
    kd: np.ndarray,
) -> np.ndarray:
    """Compute PD control torques."""
    return (target_q - q) * kp + (target_dq - dq) * kd


class DeployMuJoCo:
    """Deploy RL policy to MuJoCo simulation."""

    def __init__(self, config_path: str):
        self.config_path = Path(config_path)
        self.project_root = get_project_root()
        self._load_config()
        self._load_model()
        self._load_policy()
        self._init_state()

    def _load_config(self):
        """Load configuration from YAML files."""
        with open(self.config_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        # Path configuration
        policy_path = config["policy_path"]
        if "{PROJECT_ROOT}" in policy_path:
            policy_path = policy_path.replace("{PROJECT_ROOT}", str(self.project_root))
        self.policy_path = Path(policy_path)

        xml_path = config["xml_path"]
        if "{PROJECT_ROOT}" in xml_path:
            xml_path = xml_path.replace("{PROJECT_ROOT}", str(self.project_root))
        self.xml_path = Path(xml_path)

        # Load robot parameters from deploy.yaml
        robot_params_path = config.get("robot_params_path")
        if robot_params_path and "{PROJECT_ROOT}" in robot_params_path:
            robot_params_path = robot_params_path.replace("{PROJECT_ROOT}", str(self.project_root))

        if robot_params_path:
            with open(robot_params_path, "r") as f:
                robot_params = yaml.load(f, Loader=yaml.FullLoader)
            self._load_robot_params(robot_params)
        else:
            raise ValueError("robot_params_path is required in config")

        # Simulation parameters
        self.simulation_duration = config.get("simulation_duration", 60.0)
        self.simulation_dt = config.get("simulation_dt", 0.002)
        self.control_decimation = config.get("control_decimation", 10)

        # Policy dimensions
        self.num_actions = config["num_actions"]
        self.num_obs = config["num_obs"]

        # Command configuration
        self.cmd = np.array(config.get("cmd_init", [0.5, 0.0, 0.0]), dtype=np.float32)

        # Gait phase (optional)
        self.gait_period = config.get("gait_period")

        # Use sensor data for observation (optional, default: False)
        self.use_sensor = config.get("use_sensor", False)

        # Policy joint names in order - used to build joint_ids_map by name matching
        # (deploy.yaml's joint_ids_map is in IsaacSim USD order, NOT MuJoCo order)
        self.policy_joint_names = config.get("joint_names")

    def _load_robot_params(self, params: dict):
        """Load robot parameters from deploy.yaml.

        Note on orderings in deploy.yaml (exported by IsaacLab's export_deploy_cfg):
        - joint_ids_map: maps IsaacSim joint index -> SDK joint position
        - stiffness, damping: in SDK order (reordered by export)
        - default_joint_pos, action offset/scale: in IsaacSim order (NOT reordered!)
        - Policy input/output: IsaacSim order

        We convert everything to IsaacSim order here, since that's what the policy expects.
        """
        sdk_kps = np.array(params["stiffness"], dtype=np.float32)
        sdk_kds = np.array(params["damping"], dtype=np.float32)

        # deploy.yaml's joint_ids_map: IsaacSim joint i -> SDK position
        self.isaac_to_sdk_map = np.array(params["joint_ids_map"]) if params.get("joint_ids_map") else None

        # Reorder kps/kds from SDK order to IsaacSim order
        if self.isaac_to_sdk_map is not None:
            self.kps = sdk_kps[self.isaac_to_sdk_map]
            self.kds = sdk_kds[self.isaac_to_sdk_map]
        else:
            self.kps = sdk_kps
            self.kds = sdk_kds

        # Default joint positions (already in IsaacSim order from deploy.yaml)
        self.default_angles = np.array(params["default_joint_pos"], dtype=np.float32)

        # Will be rebuilt in _load_model using name matching
        self.joint_ids_map = None
        self.inverse_joint_ids_map = None

        # Action scale
        action_config = params.get("actions", {}).get("JointPositionAction", {})
        self.action_scale = np.array(action_config.get("scale", 0.25), dtype=np.float32)

        # Observation scales
        obs_config = params.get("observations", {})
        self.ang_vel_scale = np.array(obs_config.get("base_ang_vel", {}).get("scale", [0.25, 0.25, 0.25]), dtype=np.float32)
        self.dof_pos_scale = np.array(obs_config.get("joint_pos_rel", {}).get("scale", 1.0), dtype=np.float32)
        self.dof_vel_scale = np.array(obs_config.get("joint_vel_rel", {}).get("scale", 0.05), dtype=np.float32)
        self.cmd_scale = np.array(obs_config.get("velocity_commands", {}).get("scale", [1.0, 1.0, 1.0]), dtype=np.float32)

        # History length from observations
        first_obs = next(iter(obs_config.values())) if obs_config else {}
        self.history_length = first_obs.get("history_length", 1)

    def _load_model(self):
        """Load MuJoCo model."""
        self.m = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.d = mujoco.MjData(self.m)
        self.m.opt.timestep = self.simulation_dt

        print(f"Model loaded from: {self.xml_path}")
        print(f"Number of joints: {self.m.njnt}(including 1 freejoint), actuators: {self.m.nu}")

        # Store total actuator count for reuse
        self.total_actuators = self.m.nu

        # Build joint_ids_map (IsaacSim -> MuJoCo) from joint names.
        # deploy.yaml's joint_ids_map is IsaacSim->SDK, not useful for MuJoCo directly.
        # We need: SDK names (from config) + isaac_to_sdk (from deploy.yaml) -> IsaacSim names -> MuJoCo match
        if self.policy_joint_names is not None and self.isaac_to_sdk_map is not None:
            sdk_names = self.policy_joint_names
            # Reconstruct IsaacSim joint names: isaac_names[i] = sdk_names[isaac_to_sdk[i]]
            isaac_names = [sdk_names[sdk_pos] for sdk_pos in self.isaac_to_sdk_map]

            mujoco_actuator_names = [self.m.actuator(i).name for i in range(self.total_actuators)]
            print(f"MuJoCo actuator names: {mujoco_actuator_names}")
            print(f"IsaacSim joint names (reconstructed): {isaac_names}")
            rebuilt = []
            for name in isaac_names:
                # Strip IsaacSim USD prefix like 'a__' if present
                clean = name[3:] if name.startswith("a__") else name
                if clean in mujoco_actuator_names:
                    rebuilt.append(mujoco_actuator_names.index(clean))
                else:
                    raise ValueError(f"IsaacSim joint '{name}' (clean='{clean}') not found in MuJoCo actuators")
            self.joint_ids_map = np.array(rebuilt, dtype=np.int64)
            print(f"joint_ids_map (IsaacSim -> MuJoCo): {self.joint_ids_map}")

        # Process joint mapping and compute missing count once
        self.missing_count = 0
        if self.joint_ids_map is not None:
            existing_indices = set(self.joint_ids_map)
            all_indices = set(range(self.total_actuators))
            missing_indices = sorted(all_indices - existing_indices)
            self.missing_count = len(missing_indices)

            if self.missing_count > 0:
                print(f"Joint map missing indices: {missing_indices}, appending to end for inverse mapping")
                extended_joint_ids_map = np.concatenate([self.joint_ids_map, np.array(missing_indices)])
                self.extended_default_angles = np.concatenate([self.default_angles, np.zeros(self.missing_count, dtype=np.float32)])
                extended_kps = np.concatenate([self.kps, np.zeros(self.missing_count, dtype=np.float32)])
                extended_kds = np.concatenate([self.kds, np.zeros(self.missing_count, dtype=np.float32)])
            else:
                extended_joint_ids_map = self.joint_ids_map
                self.extended_default_angles = self.default_angles
                extended_kps = self.kps
                extended_kds = self.kds

            # Build inverse_joint_ids_map: MuJoCo actuator index -> policy joint index
            self.inverse_joint_ids_map = np.zeros(self.total_actuators, dtype=np.int64)
            for i, j in enumerate(extended_joint_ids_map):
                self.inverse_joint_ids_map[j] = i

            print(f"joint_ids_map (for policy input): {self.joint_ids_map}")
            print(f"inverse_joint_ids_map: {self.inverse_joint_ids_map}")

            # Pre-compute kps/kds in MuJoCo actuator order
            self.kps_mujoco = extended_kps[self.inverse_joint_ids_map]
            self.kds_mujoco = extended_kds[self.inverse_joint_ids_map]

            initial_joint_pos = self.extended_default_angles[self.inverse_joint_ids_map]
        else:
            initial_joint_pos = self.default_angles
            self.kps_mujoco = self.kps
            self.kds_mujoco = self.kds

        # Load home keyframe if available (sets base pose + joint angles)
        if self.m.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
            print(f"Loaded keyframe 'home': base pos = {self.d.qpos[:3]}, quat = {self.d.qpos[3:7]}")
        # Override joint angles with deploy.yaml defaults (keep base pose from keyframe)
        self.d.qpos[7:] = initial_joint_pos
        mujoco.mj_forward(self.m, self.d)
        print(f"Initial joint positions set: {initial_joint_pos[:6]}... (showing first 6)")
        print(f"Use sensor data: {self.use_sensor}")

        # Initialize sensor offsets if using sensor data
        if self.use_sensor:
            self._find_sensor_offsets()

    def _find_sensor_offsets(self):
        """Find sensor data offsets by sensor name."""
        self.sensor_offsets = {}
        for i in range(self.m.nsensor):
            name = self.m.sensor(i).name
            addr = int(self.m.sensor(i).adr)
            dim = int(self.m.sensor(i).dim)
            self.sensor_offsets[name] = (addr, dim)

        if self.use_sensor:
            print(f"Sensor offsets found: {len(self.sensor_offsets)} sensors")

    def _load_policy(self):
        """Load policy (supports PyTorch .pt and ONNX .onnx formats)."""
        print(f"Loading policy from: {self.policy_path}")

        if str(self.policy_path).endswith('.pt'):
            self.policy = torch.jit.load(str(self.policy_path))
            self.policy.eval()
            self.policy_type = 'torch'
        else:
            self.policy = ort.InferenceSession(str(self.policy_path), providers=["CPUExecutionProvider"])
            self.policy_type = 'onnx'

        print(f"Policy type: {self.policy_type}")

    def _init_state(self):
        """Initialize state variables."""
        self.action = np.zeros(self.num_actions, dtype=np.float32)

        # Initialize target_dof_pos with correct size
        if self.missing_count > 0:
            self.target_dof_pos = self.extended_default_angles.copy()
        else:
            self.target_dof_pos = self.default_angles.copy()

        self.obs = np.zeros(self.num_obs, dtype=np.float32)
        self.counter = 0

        self.has_phase = self.gait_period is not None and self.gait_period > 0

        # Initialize per-term history buffers
        self.ang_vel_history = [np.zeros(3, dtype=np.float32) for _ in range(self.history_length)]
        self.gravity_history = [np.zeros(3, dtype=np.float32) for _ in range(self.history_length)]
        self.cmd_history = [np.zeros(3, dtype=np.float32) for _ in range(self.history_length)]
        self.joint_pos_history = [np.zeros(self.num_actions, dtype=np.float32) for _ in range(self.history_length)]
        self.joint_vel_history = [np.zeros(self.num_actions, dtype=np.float32) for _ in range(self.history_length)]
        self.action_history = [np.zeros(self.num_actions, dtype=np.float32) for _ in range(self.history_length)]

        self._init_history_buffers()
        print(f"History length: {self.history_length}, Has phase: {self.has_phase}")

    def _init_history_buffers(self):
        """Initialize history buffers with current state."""
        qj, dqj = self._get_joint_state()
        quat, omega = self._get_body_state()

        qj_scaled = (qj - self.default_angles) * self.dof_pos_scale
        dqj_scaled = dqj * self.dof_vel_scale
        gravity_orientation = get_gravity_orientation(quat)
        omega_scaled = omega * self.ang_vel_scale

        for i in range(self.history_length):
            self.ang_vel_history[i] = omega_scaled.copy()
            self.gravity_history[i] = gravity_orientation.copy()
            self.cmd_history[i] = (self.cmd * self.cmd_scale).copy()
            self.joint_pos_history[i] = qj_scaled.copy()
            self.joint_vel_history[i] = dqj_scaled.copy()
            self.action_history[i] = self.action.copy()

    def _get_joint_state(self):
        """Get joint positions and velocities in policy order.

        If use_sensor is True, use sensor data (sensordata) which follows sensor order in XML.
        Otherwise, use qpos/qvel which follows body joint order in XML.
        """
        if self.use_sensor:
            # Use sensor data - order follows sensor definition in XML
            qj_mujoco = self.d.sensordata[:self.total_actuators].copy()
            dqj_mujoco = self.d.sensordata[self.total_actuators:2*self.total_actuators].copy()
        else:
            # Use qpos/qvel - order follows body joint definition in XML
            qj_mujoco = self.d.qpos[7:].copy()
            dqj_mujoco = self.d.qvel[6:].copy()

        if self.joint_ids_map is not None:
            return qj_mujoco[self.joint_ids_map], dqj_mujoco[self.joint_ids_map]
        return qj_mujoco, dqj_mujoco

    def _get_body_state(self):
        """Get body state (quaternion and angular velocity).

        If use_sensor is True, use sensor data from 'imu_quat' and 'imu_gyro'.
        Otherwise, use qpos/qvel which are the ground truth values.
        """
        if self.use_sensor:
            # Get quaternion from imu_quat sensor (4D)
            if 'imu_quat' in self.sensor_offsets:
                addr, dim = self.sensor_offsets['imu_quat']
                quat = self.d.sensordata[addr:addr+dim].copy()
            else:
                quat = self.d.qpos[3:7].copy()

            # Get angular velocity from imu_gyro sensor (3D)
            if 'imu_gyro' in self.sensor_offsets:
                addr, dim = self.sensor_offsets['imu_gyro']
                omega = self.d.sensordata[addr:addr+dim].copy()
            else:
                omega = self.d.qvel[3:6].copy()
        else:
            # Use ground truth from qpos/qvel
            quat = self.d.qpos[3:7].copy()
            omega = self.d.qvel[3:6].copy()

        return quat, omega

    def _update_history_buffers(self):
        """Update all history buffers with current state."""
        qj, dqj = self._get_joint_state()
        quat, omega = self._get_body_state()

        qj_scaled = (qj - self.default_angles) * self.dof_pos_scale
        dqj_scaled = dqj * self.dof_vel_scale
        gravity_orientation = get_gravity_orientation(quat)
        omega_scaled = omega * self.ang_vel_scale

        self.ang_vel_history.pop(0)
        self.ang_vel_history.append(omega_scaled.copy())

        self.gravity_history.pop(0)
        self.gravity_history.append(gravity_orientation.copy())

        self.cmd_history.pop(0)
        self.cmd_history.append((self.cmd * self.cmd_scale).copy())

        self.joint_pos_history.pop(0)
        self.joint_pos_history.append(qj_scaled.copy())

        self.joint_vel_history.pop(0)
        self.joint_vel_history.append(dqj_scaled.copy())

        self.action_history.pop(0)
        self.action_history.append(self.action.copy())

    def _build_observation(self) -> np.ndarray:
        """Build observation from history buffers."""
        self._update_history_buffers()

        obs_parts = [
            np.concatenate(self.ang_vel_history),
            np.concatenate(self.gravity_history),
            np.concatenate(self.cmd_history),
            np.concatenate(self.joint_pos_history),
            np.concatenate(self.joint_vel_history),
            np.concatenate(self.action_history),
        ]

        if self.has_phase:
            count = self.counter * self.simulation_dt
            phase = count % self.gait_period / self.gait_period
            sin_phase = np.sin(2 * np.pi * phase)
            cos_phase = np.cos(2 * np.pi * phase)
            obs_parts.append(np.array([sin_phase, cos_phase]))

        return np.concatenate(obs_parts)

    def run(self, record_path: str | None = None, record_fps: int = 30, record_res: tuple = (1280, 720)):
        """Run the simulation.

        Args:
            record_path: If set, save video to this path (e.g. "output.mp4").
            record_fps: Video frame rate. Default 30.
            record_res: Video resolution (width, height). Default (1280, 720).
        """
        print(f"\nStarting simulation...")
        print(f"Duration: {self.simulation_duration}s, Control freq: {1.0 / (self.simulation_dt * self.control_decimation):.1f} Hz")
        print(f"Initial command: lin_vel_x={self.cmd[0]:.2f}, lin_vel_y={self.cmd[1]:.2f}, ang_vel_z={self.cmd[2]:.2f}")

        # Setup offscreen renderer for recording
        renderer = None
        video_writer = None
        record_interval = None
        if record_path:
            import imageio.v2 as imageio
            # Expand offscreen framebuffer to match desired resolution
            self.m.vis.global_.offwidth = record_res[0]
            self.m.vis.global_.offheight = record_res[1]
            renderer = mujoco.Renderer(self.m, height=record_res[1], width=record_res[0])
            video_writer = imageio.get_writer(
                record_path, fps=record_fps, codec="libx264", quality=8,
                macro_block_size=1,
            )
            record_interval = max(1, int(1.0 / (self.simulation_dt * record_fps)))
            print(f"Recording to: {record_path} ({record_res[0]}x{record_res[1]} @ {record_fps}fps)")

        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            start_time = time.time()

            while viewer.is_running() and time.time() - start_time < self.simulation_duration:
                step_start = time.time()

                # Apply PD control
                if self.inverse_joint_ids_map is not None:
                    target_dof_pos_mujoco = self.target_dof_pos[self.inverse_joint_ids_map]
                else:
                    target_dof_pos_mujoco = self.target_dof_pos

                # Get current joint state for PD control
                if self.use_sensor:
                    qj_mujoco = self.d.sensordata[:self.total_actuators].copy()
                    dqj_mujoco = self.d.sensordata[self.total_actuators:2*self.total_actuators].copy()
                else:
                    qj_mujoco = self.d.qpos[7:].copy()
                    dqj_mujoco = self.d.qvel[6:].copy()

                tau = pd_control(
                    target_dof_pos_mujoco,
                    qj_mujoco,
                    self.kps_mujoco,
                    np.zeros_like(self.kds_mujoco),
                    dqj_mujoco,
                    self.kds_mujoco,
                )
                self.d.ctrl[:] = tau

                mujoco.mj_step(self.m, self.d)
                self.counter += 1

                # Record frame (synced with viewer camera)
                if video_writer is not None and self.counter % record_interval == 0:
                    renderer.update_scene(self.d, camera=viewer.cam)
                    frame = renderer.render()  # RGB uint8 HxWx3
                    video_writer.append_data(frame)

                # Policy inference at control frequency
                if self.counter % self.control_decimation == 0:
                    obs = self._build_observation()

                    if self.policy_type == 'torch':
                        obs_tensor = torch.from_numpy(obs).unsqueeze(0).float()
                        with torch.no_grad():
                            action = self.policy(obs_tensor).squeeze().numpy()
                    else:
                        obs_tensor = obs.reshape(1, -1).astype(np.float32)
                        action = self.policy.run(None, {"obs": obs_tensor})[0].squeeze()

                    self.action = action.astype(np.float32)
                    target_dof_pos_policy = self.action * self.action_scale + self.default_angles

                    # Pad zeros for missing joints
                    if self.missing_count > 0:
                        self.target_dof_pos = np.concatenate([
                            target_dof_pos_policy,
                            np.zeros(self.missing_count, dtype=np.float32)
                        ])
                    else:
                        self.target_dof_pos = target_dof_pos_policy
                    viewer.sync()
                    #time.sleep(1)

                time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

        if video_writer is not None:
            video_writer.close()
            print(f"Video saved to: {record_path}")
        if renderer is not None:
            renderer.close()
        print("Simulation finished.")


def main():
    parser = argparse.ArgumentParser(description="Deploy RL policy to MuJoCo simulation")
    parser.add_argument("config_file", type=str, help="Config file name or absolute path")
    parser.add_argument("--record", type=str, default=None, help="Output video path (e.g. output.mp4)")
    parser.add_argument("--record_fps", type=int, default=30, help="Video frame rate (default: 30)")
    parser.add_argument("--record_res", type=int, nargs=2, default=[1280, 720], help="Video resolution W H (default: 1280 720)")
    args = parser.parse_args()

    if Path(args.config_file).is_absolute():
        config_path = args.config_file
    else:
        config_path = Path(__file__).parent / "configs" / args.config_file

    if not Path(config_path).exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    deploy = DeployMuJoCo(str(config_path))
    deploy.run(record_path=args.record, record_fps=args.record_fps, record_res=tuple(args.record_res))


if __name__ == "__main__":
    main()
