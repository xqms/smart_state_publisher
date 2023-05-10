smart_state_publisher
=====================

In hybrid setups with multiple hardware drivers, we usually use a combination
of [joint_state_publisher] and [robot_state_publisher] to publish transforms.
`joint_state_publisher` takes care of merging the different `joint_state`
topics, and `robot_state_publisher` computes the forward kinematics and
publishes `tf` transforms.

However, both `joint_state_publisher` and `robot_state_publisher` operate with
fixed rates, introducing latency. If you need transforms *fast*, this is
undesirable.

This node combines the effect of both nodes and does not use a fixed update
rate. Instead, multiple `sensor_msgs/JointState` sources can be specified, each
with a *rate limit*. Whenever a joint state message is received, the
corresponding transforms are updated and sent out.

Example launch file:

```XML
<node name="state_pub" pkg="smart_state_publisher" type="smart_state_publisher" respawn="true">
<rosparam>
# Set to true to get latency information printed
diagnostics: false

# Set to true to send out updates for *all* tfs for
# every received joint state message. This is very inefficient,
# but allows quick prototyping if not all joint state sources
# are online.
publish_all_joints: false

# List of joint state sources
sources:
  - topic: right/joint_states
    # Rate limit in Hz
    rate: 100.0
  - topic: left/joint_states
    rate: 100.0
  - topic: head/joint_states
    rate: 0.0
  - topic: /anna/left/sih/joint_states
  - topic: /anna/svh_controller/channel_feedback_tuned
  - topic: /anna/atlas_spine/joint_states
</rosparam>
</node>
```

[joint_state_publisher]: http://wiki.ros.org/joint_state_publisher
[robot_state_publisher]: http://wiki.ros.org/robot_state_publisher
