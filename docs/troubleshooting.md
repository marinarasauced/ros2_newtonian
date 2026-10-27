# Troubleshooting

- [Parameter file](#parameter-file)
- [Build](#build)
- [Runtime](#runtime)

## Parameter file

| Problem | Cause | Fix |
|---|---|---|
| `Couldn't parse params file ... Sequence should be of same type` | a list has integers and floats, for example `[0, -3.0]` | write all numbers with the same type: `[0.0, -3.0]` |
| `Couldn't parse params file ... Cannot have a value before ros__parameters` | an empty map `{}` | remove the key |
| `Cannot declare a statically typed parameter with default value of type PARAMETER_NOT_SET` | an empty list `[]` | remove the key |
| `ParameterError: <key> encountered unknown key ...` | spelling error in a top-level or `sim` key | correct the key in the message |
| `ParameterError: <key> expected ... but got ...` | wrong type or value | correct the value of the key in the message |
| `ParameterError: ... matches no joint` | a glob in `drives` or `initial_joint_q` selects no joint | use the short joint names from the robot file |
| `ParameterError: ... matches no shape` | a glob in `colors` selects no shape | use the short shape names from the robot file |
| `ParameterError: ... set 'robot' to one of ...` | more than one robot, and a sensor or input has no `robot` key | add `robot: <name>` |
| A setting in an entry has no effect | spelling error in an entry key; these keys are not checked | compare the key with [Configuration](configuration.md) |

## Build

| Problem | Cause | Fix |
|---|---|---|
| `KeyError: <kind> type '...' is unknown` | wrong `type` in an entry | use a type from [Configuration](configuration.md) |
| `solver.type '...' does not report contact forces, which contact sensors need` | contact sensor with `kamino` and `use_collision_detector: true` | remove `use_collision_detector`, or use a different solver |
| `solver.type '...' does not support lazy drives` | `lazy: true` with `kamino` | remove `lazy`, or use a different solver |
| `robots has 2 entries but only one robot per sim is supported` | more than one robot | use one robot |
| `error: can't copy '.../build/newtonian/...': doesn't exist or not a regular file` | with `--symlink-install`, `setup.py` runs from `build/newtonian`, which holds links from the earlier build to files that were since moved or deleted, for example in `descriptions/`, `config/` or `launch/` | `rm -rf build/newtonian install/newtonian`, then build again |
| The viewer does not open on Wayland | GLFW uses Wayland | `export PYGLFW_LIBRARY_VARIANT=x11` |
| `Couldn't find function overload for 'contact_force_fn'`, after `RuntimeWarning: MuJoCo dependency version mismatch with Newton's declared requirements` | `mujoco` or `mujoco_warp` is not the version that Newton needs | `pip install -r requirements.txt` |

## Runtime

| Problem | Cause | Fix |
|---|---|---|
| RViz: "incompatible QoS" on a sensor topic | sensors are best effort, RViz subscribes reliable | set the RViz display to *Best Effort*, or set `qos: "default"` on the sensor |
| No messages arrive on an input | the publisher is best effort, the input is reliable | publish reliable, or set `qos: "sensor_data"` on the input |
| The racecar stops after 0.5 s | the `racecar` input timed out | publish at a rate above 2 Hz, for example `ros2 topic pub -r 10 ...`, or change `timeout` |
| A lazy drive does not move | no message on an input of this robot since the start or the last reset | publish a command |
| Other nodes do not see time pass, or TF is old | they use wall-clock time | set `use_sim_time: true` in these nodes |
| `real-time factor ... differs from the target ...` | one frame takes longer than `1 / (frame_rate × real_time_factor)` | decrease `substeps`, use a faster solver, or decrease `real_time_factor` |
| The robot is not in the viewer | the simulation is unstable (NaN), usually `featherstone` with few substeps | increase `body_armature` or `substeps` |
| `rebuild` returns `success: false` | the new parameters have an error | read `message` in the response; the old scene continues to run |
