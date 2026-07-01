#!/usr/bin/env python3
"""Apply GA-OCP MuJoCo closed-loop instrumentation for reviewer experiments.

GA-OCP is outside this workspace's writable sandbox root in this session.  This
script keeps the external edits reproducible and idempotent.
"""

from __future__ import annotations

from pathlib import Path


GA_ROOT = Path("/home/chenwh/ros2_ws/src/GA-OCP")
ROS2_ROOT = GA_ROOT / "ga_ocp_ros2"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"Unable to find replacement anchor for {label}")
    return text.replace(old, new, 1)


def update_joint_executor() -> None:
    path = ROS2_ROOT / "scripts" / "joint_command_executor.py"
    text = path.read_text(encoding="utf-8")

    text = replace_once(
        text,
        "#!/usr/bin/env python3\n"
        "import time\n",
        "#!/usr/bin/env python3\n"
        "import signal\n"
        "import time\n",
        "executor signal import",
    )

    text = replace_once(
        text,
        "from sensor_msgs.msg import JointState\n"
        "from trajectory_msgs.msg import JointTrajectory\n",
        "from sensor_msgs.msg import JointState\n"
        "from std_msgs.msg import String\n"
        "from trajectory_msgs.msg import JointTrajectory\n",
        "executor status import",
    )

    text = replace_once(
        text,
        "def _array(values: List[float]) -> np.ndarray:\n"
        "    return np.asarray(values, dtype=float)\n",
        "def _array(values: List[float]) -> np.ndarray:\n"
        "    return np.asarray(values, dtype=float)\n"
        "\n"
        "\n"
        "_PARAM_SENTINEL = 1.0e100\n"
        "\n"
        "\n"
        "def _vector3_from_params(\n"
        "    node: Node,\n"
        "    vector_name: str,\n"
        "    scalar_prefix: str,\n"
        "    default: List[float],\n"
        ") -> np.ndarray:\n"
        "    value = np.asarray(node.declare_parameter(vector_name, default).value, dtype=float)\n"
        "    if value.shape != (3,):\n"
        "        raise ValueError(f'{vector_name} must contain exactly 3 values.')\n"
        "\n"
        "    scalar_values = [\n"
        "        float(node.declare_parameter(f'{scalar_prefix}_{axis}', _PARAM_SENTINEL).value)\n"
        "        for axis in ('x', 'y', 'z')\n"
        "    ]\n"
        "    scalar_set = [abs(v) < 0.5 * _PARAM_SENTINEL for v in scalar_values]\n"
        "    if any(scalar_set):\n"
        "        if not all(scalar_set):\n"
        "            raise ValueError(f'{scalar_prefix}_x/y/z must be set together.')\n"
        "        value = np.asarray(scalar_values, dtype=float)\n"
        "    return value\n",
        "executor vector3 helper",
    )

    text = replace_once(
        text,
        "        self.mass_scale = float(self.declare_parameter('mass_scale', 1.0).value)\n"
        "        self.payload_mass = float(self.declare_parameter('payload_mass', 0.0).value)\n"
        "        self.payload_body_name = str(self.declare_parameter('payload_body_name', 'attachment').value)\n"
        "        self.payload_com = np.asarray(\n"
        "            self.declare_parameter('payload_com', [0.0, 0.0, 0.05]).value, dtype=float\n"
        "        )\n"
        "        if self.payload_com.shape != (3,):\n"
        "            raise ValueError('payload_com must contain exactly 3 values.')\n",
        "        self.mass_scale = float(self.declare_parameter('mass_scale', 1.0).value)\n"
        "        self.payload_mass = float(self.declare_parameter('payload_mass', 0.0).value)\n"
        "        self.payload_body_name = str(self.declare_parameter('payload_body_name', 'attachment').value)\n"
        "        self.payload_com = _vector3_from_params(\n"
        "            self, 'payload_com', 'payload_com', [0.0, 0.0, 0.05]\n"
        "        )\n"
        "        self.enable_viewer = bool(self.declare_parameter('enable_viewer', True).value)\n"
        "        self.external_force_body_name = str(\n"
        "            self.declare_parameter('external_force_body_name', 'wrist_3_link').value\n"
        "        )\n"
        "        self.external_force_start_s = float(\n"
        "            self.declare_parameter('external_force_start_s', -1.0).value\n"
        "        )\n"
        "        self.external_force_duration_s = float(\n"
        "            self.declare_parameter('external_force_duration_s', 0.0).value\n"
        "        )\n"
        "        self.external_force = _vector3_from_params(\n"
        "            self, 'external_force', 'external_force', [0.0, 0.0, 0.0]\n"
        "        )\n"
        "        self.external_torque = _vector3_from_params(\n"
        "            self, 'external_torque', 'external_torque', [0.0, 0.0, 0.0]\n"
        "        )\n",
        "executor parameter block",
    )

    text = replace_once(
        text,
        "        self.ctrl_addrs: List[int] = []\n",
        "        self.ctrl_addrs: List[int] = []\n"
        "        self.external_force_body_id: Optional[int] = None\n",
        "executor external body id member",
    )

    text = replace_once(
        text,
        "        self.publish_joint_state = self.create_publisher(JointState, self.joint_state_topic, 20)\n"
        "        self.create_subscription(JointTrajectory, self.command_topic, self.trajectory_callback, 10)\n",
        "        self.publish_joint_state = self.create_publisher(JointState, self.joint_state_topic, 20)\n"
        "        self.create_subscription(JointTrajectory, self.command_topic, self.trajectory_callback, 10)\n"
        "        self.create_subscription(String, '/planning_status', self.status_callback, 10)\n",
        "executor status subscription",
    )

    text = replace_once(
        text,
        "        self.effort_target = np.zeros(self.n, dtype=float)\n"
        "\n"
        "        self.paused = False\n",
        "        self.effort_target = np.zeros(self.n, dtype=float)\n"
        "\n"
        "        self.stop_requested = False\n"
        "        self.paused = False\n",
        "executor stop requested member",
    )

    text = replace_once(
        text,
        "            f\"model={self.xml_file}, cmd={self.command_topic}, state={self.joint_state_topic}, \"\n"
        "            f\"mass_scale={self.mass_scale:.3f}, payload_mass={self.payload_mass:.3f}\"\n",
        "            f\"model={self.xml_file}, cmd={self.command_topic}, state={self.joint_state_topic}, \"\n"
        "            f\"mass_scale={self.mass_scale:.3f}, payload_mass={self.payload_mass:.3f}, \"\n"
        "            f\"payload_com={self.payload_com.tolist()}, viewer={self.enable_viewer}, \"\n"
        "            f\"external_body={self.external_force_body_name}, \"\n"
        "            f\"external_start={self.external_force_start_s:.3f}, \"\n"
        "            f\"external_duration={self.external_force_duration_s:.3f}, \"\n"
        "            f\"external_force={self.external_force.tolist()}, \"\n"
        "            f\"external_torque={self.external_torque.tolist()}\"\n",
        "executor startup log",
    )

    text = replace_once(
        text,
        "        if dirty:\n"
        "            mujoco.mj_setConst(model, data)\n"
        "\n"
        "    def _lookup_joint_addresses(self, model: mujoco.MjModel) -> None:\n",
        "        if dirty:\n"
        "            mujoco.mj_setConst(model, data)\n"
        "\n"
        "    def _lookup_external_force_body(self, model: mujoco.MjModel) -> None:\n"
        "        has_force = np.linalg.norm(self.external_force) > 0.0\n"
        "        has_torque = np.linalg.norm(self.external_torque) > 0.0\n"
        "        if self.external_force_duration_s <= 0.0 or not (has_force or has_torque):\n"
        "            self.external_force_body_id = None\n"
        "            return\n"
        "        self.external_force_body_id = self._lookup_body_id(model, self.external_force_body_name)\n"
        "        self.get_logger().info(\n"
        "            f\"External wrench enabled on body '{self.external_force_body_name}' \"\n"
        "            f\"from t={self.external_force_start_s:.3f}s for \"\n"
        "            f\"{self.external_force_duration_s:.3f}s.\"\n"
        "        )\n"
        "\n"
        "    def _apply_external_wrench(self, data: mujoco.MjData) -> None:\n"
        "        data.xfrc_applied[:, :] = 0.0\n"
        "        if self.external_force_body_id is None:\n"
        "            return\n"
        "        if data.time < self.external_force_start_s:\n"
        "            return\n"
        "        if data.time >= self.external_force_start_s + self.external_force_duration_s:\n"
        "            return\n"
        "        # MuJoCo spatial wrench order is torque first, then force.\n"
        "        data.xfrc_applied[self.external_force_body_id, 0:3] = self.external_torque\n"
        "        data.xfrc_applied[self.external_force_body_id, 3:6] = self.external_force\n"
        "\n"
        "    def _lookup_joint_addresses(self, model: mujoco.MjModel) -> None:\n",
        "executor external wrench methods",
    )

    text = replace_once(
        text,
        "    def trajectory_callback(self, msg: JointTrajectory) -> None:\n",
        "    def status_callback(self, msg: String) -> None:\n"
        "        if 'finished' in msg.data:\n"
        "            self.stop_requested = True\n"
        "\n"
        "    def trajectory_callback(self, msg: JointTrajectory) -> None:\n",
        "executor status callback",
    )

    if "    def _run_loop(self, model: mujoco.MjModel, data: mujoco.MjData, viewer=None) -> None:\n" not in text:
        text = replace_once(
            text,
            "    def run(self) -> None:\n"
            "        model = mujoco.MjModel.from_xml_path(self.xml_file)\n"
            "        data = mujoco.MjData(model)\n"
            "        self._apply_runtime_model_variations(model, data)\n"
            "        self._lookup_joint_addresses(model)\n"
            "        data.ctrl[self.ctrl_addrs] = self.default_target\n"
            "\n"
            "        with mujoco.viewer.launch_passive(model, data, key_callback=self.key_callback) as viewer:\n"
            "            while viewer.is_running() and rclpy.ok():\n"
            "                step_start = time.time()\n"
            "\n"
            "                self._sample_active_command(step_start)\n"
            "                current_position = self._publish_joint_state(data)\n"
            "                position_error = np.max(np.abs(current_position - self.position_target))\n"
            "\n"
            "                if not self.paused:\n"
            "                    self._apply_control(data)\n"
            "                    mujoco.mj_step(model, data)\n"
            "                    viewer.sync()\n"
            "\n"
            "                if position_error < self.position_tolerance and not self.trajectory:\n"
            "                    self.position_target = current_position.copy()\n"
            "\n"
            "                rclpy.spin_once(self, timeout_sec=0.0)\n"
            "\n"
            "                elapsed = time.time() - step_start\n"
            "                sleep_time = model.opt.timestep - elapsed\n"
            "                if sleep_time > 0.0:\n"
            "                    time.sleep(sleep_time)\n",
            "    def _run_loop(self, model: mujoco.MjModel, data: mujoco.MjData, viewer=None) -> None:\n"
            "        while rclpy.ok() and (viewer is None or viewer.is_running()):\n"
            "            step_start = time.time()\n"
            "\n"
            "            self._sample_active_command(step_start)\n"
            "            current_position = self._publish_joint_state(data)\n"
            "            position_error = np.max(np.abs(current_position - self.position_target))\n"
            "\n"
            "            if not self.paused:\n"
            "                self._apply_control(data)\n"
            "                self._apply_external_wrench(data)\n"
            "                mujoco.mj_step(model, data)\n"
            "                if viewer is not None:\n"
            "                    viewer.sync()\n"
            "\n"
            "            if position_error < self.position_tolerance and not self.trajectory:\n"
            "                self.position_target = current_position.copy()\n"
            "\n"
            "            rclpy.spin_once(self, timeout_sec=0.0)\n"
            "\n"
            "            elapsed = time.time() - step_start\n"
            "            sleep_time = model.opt.timestep - elapsed\n"
            "            if sleep_time > 0.0:\n"
            "                time.sleep(sleep_time)\n"
            "\n"
            "    def run(self) -> None:\n"
            "        model = mujoco.MjModel.from_xml_path(self.xml_file)\n"
            "        data = mujoco.MjData(model)\n"
            "        self._apply_runtime_model_variations(model, data)\n"
            "        self._lookup_external_force_body(model)\n"
            "        self._lookup_joint_addresses(model)\n"
            "        data.ctrl[self.ctrl_addrs] = self.default_target\n"
            "\n"
            "        if self.enable_viewer:\n"
            "            with mujoco.viewer.launch_passive(model, data, key_callback=self.key_callback) as viewer:\n"
            "                self._run_loop(model, data, viewer)\n"
            "        else:\n"
            "            self._run_loop(model, data, None)\n",
            "executor run loop",
        )

    text = replace_once(
        text,
        "        while rclpy.ok() and (viewer is None or viewer.is_running()):\n",
        "        while rclpy.ok() and not self.stop_requested and (viewer is None or viewer.is_running()):\n",
        "executor stop requested loop condition",
    )

    text = replace_once(
        text,
        "    finally:\n"
        "        node.destroy_node()\n"
        "        rclpy.shutdown()\n",
        "    finally:\n"
        "        node.destroy_node()\n"
        "        if rclpy.ok():\n"
        "            rclpy.shutdown()\n",
        "executor clean shutdown",
    )

    text = replace_once(
        text,
        "def main() -> None:\n"
        "    rclpy.init()\n"
        "    node = MujocoJointExecutor()\n"
        "    try:\n",
        "def main() -> None:\n"
        "    rclpy.init()\n"
        "    node = MujocoJointExecutor()\n"
        "\n"
        "    def request_stop(signum, frame):\n"
        "        del signum, frame\n"
        "        node.stop_requested = True\n"
        "\n"
        "    signal.signal(signal.SIGINT, request_stop)\n"
        "    signal.signal(signal.SIGTERM, request_stop)\n"
        "\n"
        "    try:\n",
        "executor signal handlers",
    )

    path.write_text(text, encoding="utf-8")


def update_ur_launch() -> None:
    path = ROS2_ROOT / "launch" / "ga_ocp_mujoco_closed_loop_ur.launch.py"
    text = path.read_text(encoding="utf-8")
    new_text = '''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    ros2_share = get_package_share_directory('ga_ocp_ros2')
    config = f"{ros2_share}/config/closed_loop_mpc_ur.yaml"

    backend = LaunchConfiguration('backend')
    solve_budget_ms = LaunchConfiguration('solve_budget_ms')
    duration_s = LaunchConfiguration('duration_s')
    mass_scale = LaunchConfiguration('mass_scale')
    plant_payload_mass = LaunchConfiguration('plant_payload_mass')
    controller_payload_mass = LaunchConfiguration('controller_payload_mass')
    model_payload = LaunchConfiguration('model_payload')
    output_prefix = LaunchConfiguration('output_prefix')
    enable_viewer = LaunchConfiguration('enable_viewer')
    payload_body_name = LaunchConfiguration('payload_body_name')
    external_force_body_name = LaunchConfiguration('external_force_body_name')
    external_force_start_s = LaunchConfiguration('external_force_start_s')
    external_force_duration_s = LaunchConfiguration('external_force_duration_s')

    plant_payload_com_x = LaunchConfiguration('plant_payload_com_x')
    plant_payload_com_y = LaunchConfiguration('plant_payload_com_y')
    plant_payload_com_z = LaunchConfiguration('plant_payload_com_z')
    controller_payload_com_x = LaunchConfiguration('controller_payload_com_x')
    controller_payload_com_y = LaunchConfiguration('controller_payload_com_y')
    controller_payload_com_z = LaunchConfiguration('controller_payload_com_z')
    external_force_x = LaunchConfiguration('external_force_x')
    external_force_y = LaunchConfiguration('external_force_y')
    external_force_z = LaunchConfiguration('external_force_z')
    external_torque_x = LaunchConfiguration('external_torque_x')
    external_torque_y = LaunchConfiguration('external_torque_y')
    external_torque_z = LaunchConfiguration('external_torque_z')

    closed_loop_node = Node(
        package='ga_ocp_ros2',
        executable='closed_loop_mpc_node',
        name='closed_loop_mpc_node',
        output='screen',
        parameters=[
            config,
            {
                'backend': backend,
                'solve_budget_ms': ParameterValue(solve_budget_ms, value_type=float),
                'experiment_duration_s': ParameterValue(duration_s, value_type=float),
                'plant_mass_scale': ParameterValue(mass_scale, value_type=float),
                'plant_payload_mass': ParameterValue(plant_payload_mass, value_type=float),
                'controller_payload_mass': ParameterValue(controller_payload_mass, value_type=float),
                'model_payload': ParameterValue(model_payload, value_type=bool),
                'plant_payload_com_x': ParameterValue(plant_payload_com_x, value_type=float),
                'plant_payload_com_y': ParameterValue(plant_payload_com_y, value_type=float),
                'plant_payload_com_z': ParameterValue(plant_payload_com_z, value_type=float),
                'payload_com_attachment_x': ParameterValue(controller_payload_com_x, value_type=float),
                'payload_com_attachment_y': ParameterValue(controller_payload_com_y, value_type=float),
                'payload_com_attachment_z': ParameterValue(controller_payload_com_z, value_type=float),
                'external_force_body_name': external_force_body_name,
                'external_force_start_s': ParameterValue(external_force_start_s, value_type=float),
                'external_force_duration_s': ParameterValue(external_force_duration_s, value_type=float),
                'external_force_x': ParameterValue(external_force_x, value_type=float),
                'external_force_y': ParameterValue(external_force_y, value_type=float),
                'external_force_z': ParameterValue(external_force_z, value_type=float),
                'external_torque_x': ParameterValue(external_torque_x, value_type=float),
                'external_torque_y': ParameterValue(external_torque_y, value_type=float),
                'external_torque_z': ParameterValue(external_torque_z, value_type=float),
                'output_prefix': output_prefix,
            },
        ],
    )

    mujoco_executor_node = Node(
        package='ga_ocp_ros2',
        executable='joint_command_executor.py',
        name='mujoco_joint_executor_node',
        output='screen',
        parameters=[
            {
                'robot': 'ur',
                'mass_scale': ParameterValue(mass_scale, value_type=float),
                'payload_mass': ParameterValue(plant_payload_mass, value_type=float),
                'payload_body_name': payload_body_name,
                'payload_com_x': ParameterValue(plant_payload_com_x, value_type=float),
                'payload_com_y': ParameterValue(plant_payload_com_y, value_type=float),
                'payload_com_z': ParameterValue(plant_payload_com_z, value_type=float),
                'enable_viewer': ParameterValue(enable_viewer, value_type=bool),
                'external_force_body_name': external_force_body_name,
                'external_force_start_s': ParameterValue(external_force_start_s, value_type=float),
                'external_force_duration_s': ParameterValue(external_force_duration_s, value_type=float),
                'external_force_x': ParameterValue(external_force_x, value_type=float),
                'external_force_y': ParameterValue(external_force_y, value_type=float),
                'external_force_z': ParameterValue(external_force_z, value_type=float),
                'external_torque_x': ParameterValue(external_torque_x, value_type=float),
                'external_torque_y': ParameterValue(external_torque_y, value_type=float),
                'external_torque_z': ParameterValue(external_torque_z, value_type=float),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='tetrapga'),
        DeclareLaunchArgument('solve_budget_ms', default_value='10.0'),
        DeclareLaunchArgument('duration_s', default_value='20.0'),
        DeclareLaunchArgument('mass_scale', default_value='1.0'),
        DeclareLaunchArgument('plant_payload_mass', default_value='0.0'),
        DeclareLaunchArgument('controller_payload_mass', default_value='0.0'),
        DeclareLaunchArgument('model_payload', default_value='false'),
        DeclareLaunchArgument('plant_payload_com_x', default_value='0.0'),
        DeclareLaunchArgument('plant_payload_com_y', default_value='0.0'),
        DeclareLaunchArgument('plant_payload_com_z', default_value='0.05'),
        DeclareLaunchArgument('controller_payload_com_x', default_value='0.0'),
        DeclareLaunchArgument('controller_payload_com_y', default_value='0.0'),
        DeclareLaunchArgument('controller_payload_com_z', default_value='0.05'),
        DeclareLaunchArgument('payload_body_name', default_value='attachment'),
        DeclareLaunchArgument('enable_viewer', default_value='true'),
        DeclareLaunchArgument('external_force_body_name', default_value='wrist_3_link'),
        DeclareLaunchArgument('external_force_start_s', default_value='-1.0'),
        DeclareLaunchArgument('external_force_duration_s', default_value='0.0'),
        DeclareLaunchArgument('external_force_x', default_value='0.0'),
        DeclareLaunchArgument('external_force_y', default_value='0.0'),
        DeclareLaunchArgument('external_force_z', default_value='0.0'),
        DeclareLaunchArgument('external_torque_x', default_value='0.0'),
        DeclareLaunchArgument('external_torque_y', default_value='0.0'),
        DeclareLaunchArgument('external_torque_z', default_value='0.0'),
        DeclareLaunchArgument('output_prefix', default_value=''),
        closed_loop_node,
        mujoco_executor_node,
    ])
'''
    if text != new_text:
        path.write_text(new_text, encoding="utf-8")


def update_closed_loop_node() -> None:
    path = ROS2_ROOT / "src" / "closed_loop_mpc_node.cpp"
    text = path.read_text(encoding="utf-8")

    text = replace_once(
        text,
        "#include <algorithm>\n"
        "#include <chrono>\n",
        "#include <algorithm>\n"
        "#include <array>\n"
        "#include <chrono>\n",
        "closed loop include array",
    )

    text = replace_once(
        text,
        "Eigen::Vector3d ParseVector3Param(const std::vector<double>& values, const std::string& name) {\n"
        "  if (values.size() != 3) {\n"
        "    throw std::invalid_argument(name + \" must contain exactly 3 values\");\n"
        "  }\n"
        "  return Eigen::Vector3d(values[0], values[1], values[2]);\n"
        "}\n",
        "Eigen::Vector3d ParseVector3Param(const std::vector<double>& values, const std::string& name) {\n"
        "  if (values.size() != 3) {\n"
        "    throw std::invalid_argument(name + \" must contain exactly 3 values\");\n"
        "  }\n"
        "  return Eigen::Vector3d(values[0], values[1], values[2]);\n"
        "}\n"
        "\n"
        "Eigen::Vector3d DeclareVector3Param(\n"
        "    rclcpp::Node& node, const std::string& vector_name,\n"
        "    const std::array<std::string, 3>& scalar_names,\n"
        "    const std::vector<double>& default_value) {\n"
        "  Eigen::Vector3d value = ParseVector3Param(\n"
        "      node.declare_parameter<std::vector<double>>(vector_name, default_value), vector_name);\n"
        "  std::array<double, 3> scalars{};\n"
        "  std::array<bool, 3> scalar_set{};\n"
        "  for (std::size_t i = 0; i < scalar_names.size(); ++i) {\n"
        "    scalars[i] = node.declare_parameter<double>(\n"
        "        scalar_names[i], std::numeric_limits<double>::quiet_NaN());\n"
        "    scalar_set[i] = std::isfinite(scalars[i]);\n"
        "  }\n"
        "  const bool any_set = scalar_set[0] || scalar_set[1] || scalar_set[2];\n"
        "  const bool all_set = scalar_set[0] && scalar_set[1] && scalar_set[2];\n"
        "  if (any_set && !all_set) {\n"
        "    throw std::invalid_argument(vector_name + \" scalar x/y/z overrides must be set together\");\n"
        "  }\n"
        "  if (all_set) {\n"
        "    value = Eigen::Vector3d(scalars[0], scalars[1], scalars[2]);\n"
        "  }\n"
        "  return value;\n"
        "}\n",
        "closed loop vector3 scalar override helper",
    )

    text = replace_once(
        text,
        "    payload_com_attachment_ = ParseVector3Param(\n"
        "        this->declare_parameter<std::vector<double>>(\"payload_com_attachment\", {0.0, 0.0, 0.05}),\n"
        "        \"payload_com_attachment\");\n",
        "    plant_payload_com_attachment_ = DeclareVector3Param(\n"
        "        *this, \"plant_payload_com_attachment\",\n"
        "        {\"plant_payload_com_x\", \"plant_payload_com_y\", \"plant_payload_com_z\"},\n"
        "        {0.0, 0.0, 0.05});\n"
        "    payload_com_attachment_ = DeclareVector3Param(\n"
        "        *this, \"payload_com_attachment\",\n"
        "        {\"payload_com_attachment_x\", \"payload_com_attachment_y\", \"payload_com_attachment_z\"},\n"
        "        {0.0, 0.0, 0.05});\n"
        "    external_force_ = DeclareVector3Param(\n"
        "        *this, \"external_force\",\n"
        "        {\"external_force_x\", \"external_force_y\", \"external_force_z\"},\n"
        "        {0.0, 0.0, 0.0});\n"
        "    external_torque_ = DeclareVector3Param(\n"
        "        *this, \"external_torque\",\n"
        "        {\"external_torque_x\", \"external_torque_y\", \"external_torque_z\"},\n"
        "        {0.0, 0.0, 0.0});\n"
        "    external_force_body_name_ = this->declare_parameter<std::string>(\n"
        "        \"external_force_body_name\", \"wrist_3_link\");\n"
        "    external_force_start_s_ = this->declare_parameter<double>(\"external_force_start_s\", -1.0);\n"
        "    external_force_duration_s_ = this->declare_parameter<double>(\"external_force_duration_s\", 0.0);\n",
        "closed loop metadata params",
    )

    text = replace_once(
        text,
        "    RCLCPP_INFO(this->get_logger(),\n"
        "                \"Closed-loop MPC node ready. robot=%s backend=%s budget=%.3f ms horizon=%d \"\n"
        "                \"dt=%.3f control_rate=%.1fHz enforce_budget=%s output=%s\",\n"
        "                robot_config_.robot.c_str(), BackendName(backend_).c_str(), solve_budget_ms_,\n"
        "                horizon_, dt_, control_rate_hz_, enforce_solve_budget_ ? \"true\" : \"false\",\n"
        "                output_prefix_.string().c_str());\n",
        "    RCLCPP_INFO(this->get_logger(),\n"
        "                \"Closed-loop MPC node ready. robot=%s backend=%s budget=%.3f ms horizon=%d \"\n"
        "                \"dt=%.3f control_rate=%.1fHz enforce_budget=%s output=%s \"\n"
        "                \"plant_payload_com=[%s] controller_payload_com=[%s] \"\n"
        "                \"external_body=%s external_force=[%s] external_torque=[%s]\",\n"
        "                robot_config_.robot.c_str(), BackendName(backend_).c_str(), solve_budget_ms_,\n"
        "                horizon_, dt_, control_rate_hz_, enforce_solve_budget_ ? \"true\" : \"false\",\n"
        "                output_prefix_.string().c_str(),\n"
        "                FormatVector(plant_payload_com_attachment_).c_str(),\n"
        "                FormatVector(payload_com_attachment_).c_str(),\n"
        "                external_force_body_name_.c_str(),\n"
        "                FormatVector(external_force_).c_str(),\n"
        "                FormatVector(external_torque_).c_str());\n",
        "closed loop startup log metadata",
    )

    text = replace_once(
        text,
        "    out << \"robot,backend,t,tracking_error,velocity_error,torque_ratio,solve_time_ms,\"\n"
        "           \"cycle_time_ms,realtime_ratio,iterations,converged,failed,best_cost,final_stop,\"\n"
        "           \"plant_mass_scale,plant_payload_mass,controller_payload_mass,model_payload,\"\n"
        "           \"failure_message,q,dq,q_ref,dq_ref,q_cmd,dq_cmd,u_cmd,effort\\n\";\n",
        "    out << \"robot,backend,t,tracking_error,velocity_error,torque_ratio,solve_time_ms,\"\n"
        "           \"cycle_time_ms,realtime_ratio,iterations,converged,failed,best_cost,final_stop,\"\n"
        "           \"plant_mass_scale,plant_payload_mass,controller_payload_mass,model_payload,\"\n"
        "           \"plant_payload_com,controller_payload_com,payload_com_attachment,\"\n"
        "           \"external_force_body_name,external_force_start_s,external_force_duration_s,\"\n"
        "           \"external_force,external_torque,\"\n"
        "           \"failure_message,q,dq,q_ref,dq_ref,q_cmd,dq_cmd,u_cmd,effort\\n\";\n",
        "closed loop cycle header metadata",
    )

    text = replace_once(
        text,
        "          << FormatCsvNumber(controller_payload_mass_) << ','\n"
        "          << (model_payload_ ? 1 : 0) << ','\n"
        "          << CsvEscape(record.failure_message) << ','\n",
        "          << FormatCsvNumber(controller_payload_mass_) << ','\n"
        "          << (model_payload_ ? 1 : 0) << ','\n"
        "          << CsvEscape(FormatVector(plant_payload_com_attachment_)) << ','\n"
        "          << CsvEscape(FormatVector(payload_com_attachment_)) << ','\n"
        "          << CsvEscape(FormatVector(payload_com_attachment_)) << ','\n"
        "          << CsvEscape(external_force_body_name_) << ','\n"
        "          << FormatCsvNumber(external_force_start_s_) << ','\n"
        "          << FormatCsvNumber(external_force_duration_s_) << ','\n"
        "          << CsvEscape(FormatVector(external_force_)) << ','\n"
        "          << CsvEscape(FormatVector(external_torque_)) << ','\n"
        "          << CsvEscape(record.failure_message) << ','\n",
        "closed loop cycle row metadata",
    )

    text = replace_once(
        text,
        "    out << \"robot,backend,num_cycles,tracking_rmse,tracking_mean,tracking_p95,torque_ratio_mean,\"\n"
        "           \"torque_ratio_p95,torque_ratio_max,solve_time_mean_ms,solve_time_p95_ms,\"\n"
        "           \"realtime_ratio_mean,deadline_miss_rate,failure_rate,dt,horizon,solve_budget_ms,\"\n"
        "           \"control_rate_hz,experiment_duration_s,plant_mass_scale,plant_payload_mass,\"\n"
        "           \"controller_payload_mass,model_payload,payload_com_attachment\\n\";\n",
        "    out << \"robot,backend,num_cycles,tracking_rmse,tracking_mean,tracking_p95,torque_ratio_mean,\"\n"
        "           \"torque_ratio_p95,torque_ratio_max,solve_time_mean_ms,solve_time_p95_ms,\"\n"
        "           \"realtime_ratio_mean,deadline_miss_rate,failure_rate,dt,horizon,solve_budget_ms,\"\n"
        "           \"control_rate_hz,experiment_duration_s,plant_mass_scale,plant_payload_mass,\"\n"
        "           \"controller_payload_mass,model_payload,plant_payload_com,controller_payload_com,\"\n"
        "           \"payload_com_attachment,external_force_body_name,external_force_start_s,\"\n"
        "           \"external_force_duration_s,external_force,external_torque\\n\";\n",
        "closed loop summary header metadata",
    )

    text = replace_once(
        text,
        "        << FormatCsvNumber(controller_payload_mass_) << ','\n"
        "        << (model_payload_ ? 1 : 0) << ','\n"
        "        << CsvEscape(FormatVector(payload_com_attachment_)) << '\\n';\n",
        "        << FormatCsvNumber(controller_payload_mass_) << ','\n"
        "        << (model_payload_ ? 1 : 0) << ','\n"
        "        << CsvEscape(FormatVector(plant_payload_com_attachment_)) << ','\n"
        "        << CsvEscape(FormatVector(payload_com_attachment_)) << ','\n"
        "        << CsvEscape(FormatVector(payload_com_attachment_)) << ','\n"
        "        << CsvEscape(external_force_body_name_) << ','\n"
        "        << FormatCsvNumber(external_force_start_s_) << ','\n"
        "        << FormatCsvNumber(external_force_duration_s_) << ','\n"
        "        << CsvEscape(FormatVector(external_force_)) << ','\n"
        "        << CsvEscape(FormatVector(external_torque_)) << '\\n';\n",
        "closed loop summary row metadata",
    )

    text = replace_once(
        text,
        "  Eigen::Vector3d payload_com_attachment_{Eigen::Vector3d::Zero()};\n",
        "  Eigen::Vector3d plant_payload_com_attachment_{Eigen::Vector3d::Zero()};\n"
        "  Eigen::Vector3d payload_com_attachment_{Eigen::Vector3d::Zero()};\n"
        "  Eigen::Vector3d external_force_{Eigen::Vector3d::Zero()};\n"
        "  Eigen::Vector3d external_torque_{Eigen::Vector3d::Zero()};\n"
        "  std::string external_force_body_name_{\"wrist_3_link\"};\n"
        "  double external_force_start_s_{-1.0};\n"
        "  double external_force_duration_s_{0.0};\n",
        "closed loop metadata members",
    )

    path.write_text(text, encoding="utf-8")


def main() -> None:
    update_joint_executor()
    update_ur_launch()
    update_closed_loop_node()
    print("Applied GA-OCP MuJoCo closed-loop revision instrumentation.")


if __name__ == "__main__":
    main()
