# py_trees_ros_gl_viewer

Alternative to [py_trees_ros_viewer](https://github.com/splintered-reality/py_trees_ros_viewer/tree/devel), focused on high FPS for large trees. Not feature complete yet, but can display fairly large trees efficiently.

Tested on ROS2 Humble.

https://github.com/DynoRobotics/py_trees_ros_gl_viewer/assets/3470402/3d1662c6-3ab5-4255-8066-dc37d817e68c

Dependencies:

```bash
pip install pyglet>=2.0.15
```

Use:

Create your ros2 node used by the tree with `default_snapshot_stream:=true`, this will make your tree publish updates on topic `/<node_name>/snapshots` by default.

```python
node = rclpy.create_node("<node_name>", parameter_overrides=[
    rclpy.parameter.Parameter("default_snapshot_stream",
                                rclpy.parameter.Parameter.Type.BOOL, True),
    rclpy.parameter.Parameter("default_snapshot_period",
                                rclpy.parameter.Parameter.Type.DOUBLE, 1.0),
])
```

When running the viewer, use topic remapping to connect the viewer subscription to your tree node publisher.

```bash
ros2 run py_trees_ros_gl_viewer py-trees-ros-gl-viewer --ros-args -r /bt_node/snapshots:=/<node_name>/snapshots -p antialiasing:=false
```

Anti-aliasing only works well for me if I run on a computer with a graphics card.

Roadmap:

- Blackboard visualization
- Collapse sub trees
- More effient updates on tree changes
- ideas?
