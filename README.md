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
