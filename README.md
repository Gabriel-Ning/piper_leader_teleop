# piper_leader_teleop

Homomorphic Piper leader source for ROS 2 Jazzy. The first version provides:

- local libpiper 500 Hz MIT gravity compensation;
- teaching-pendant finger free move;
- fresh, coherent arm references on
  `/action_sources/piper_leader/arm/joint_reference`
  (`trajectory_msgs/JointTrajectory`);
- raw pendant state (`sensor_msgs/JointState`) and an EM-contract gripper
  reference on `/action_sources/piper_leader/end_effector/joint_reference`
  (`trajectory_msgs/JointTrajectory`, single named position-only point;
  joint position is one-finger travel = pendant opening width / 2);
- JSON source diagnostics on `/teleop/piper_leader/status`.

Haptic feedback is intentionally deferred. The follower remains independently
owned by `ros2_control`; this node only publishes leader source topics.

## Leader model

The launch file expands
`piper_description/urdf/piper_with_teach.urdf.xacro` and passes it through the
private `leader_robot_description` parameter. This is the leader gravity model
only:

- it includes the teaching-pendant inertial model;
- it contains no follower `ros2_control` hardware block;
- it is not the follower controller manager's `robot_description`;
- it is not published as the global `/robot_description` topic.

The model always includes the identity `flange_link -> flange` alias required
by `libpiper::Model`. It is canonical and unprefixed because it is private to
the teleop node rather than part of the global ROS robot description.

libpiper currently loads models from a file path, so the node materializes this
expanded parameter into a private temporary URDF while the leader is active.

## Safety and lifecycle

Launching the node is no-motion: it does not open/enable the arm until the
explicit service call.

```bash
ros2 launch piper_leader_teleop piper_leader.launch.py
ros2 service call /piper_leader/enable std_srvs/srv/SetBool '{data: true}'
```

Stop before shutting down:

```bash
ros2 service call /piper_leader/enable std_srvs/srv/SetBool '{data: false}'
```

`restore_pendant_servo_on_stop` defaults to false: stopping leaves the pendant
finger driver disabled rather than issuing an enable command with an implicit
width target.

## Fake follower validation

The left-side validation configs form this exact path:

`can0 leader -> /action_sources/piper_leader_left/arm/joint_reference (+ end_effector/joint_reference) -> em_left -> left_arm_jspc + left_gripper_fwd -> fake left follower`

Use `config/piper_leader_left.yaml` for the leader and
`piper_manipulation_controller_bringup/config/leader_fake_left.yaml` for the
follower. The leader config publishes the follower's prefixed joint names; the
leader gravity model itself remains the independent, unprefixed
`piper_with_teach` model.

Left and right leader nodes may load the same canonical model independently.
Their CAN interfaces, node namespaces, output topics, and published follower
joint names provide the ROS-side identity; libpiper model names stay unprefixed.

## Validation gates

1. Launch only. Expect a status topic with `active=false`; no action target is
   published and no hardware command is sent.
2. With the leader supported and CAN configured, enable it. Expect coherent
   arm/pendant sequences in the status JSON and moving joint references.
3. Only after gate 2 passes, configure the execution manager to consume the
   `piper_leader` source and test the follower at conservative limits.
