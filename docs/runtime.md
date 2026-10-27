# Runtime

- [Launch](#launch)
- [Topics](#topics)
- [Services](#services)
- [Change the scene at runtime](#change-the-scene-at-runtime)
- [Viewer](#viewer)

## Launch

```zsh
ros2 launch newtonian sim.launch.py config:=<scene>
```

| `config` value | File used |
|---|---|
| none | `config/racecar.yaml` of the installed package |
| a name, for example `racecar` | `config/<name>.yaml` of the installed package |
| an absolute path | that file |

A relative path is read as a name. Use an absolute path for files outside the package.

**Launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `config` | `racecar` | the scene, see above |
| `use_sim_time` | `true` | sets `use_sim_time` for the nodes of this launch. The simulator always steps on wall-clock time |

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | out, one message per frame |
| sensor topics | per sensor type, see [`sensors`](configuration.md#sensors) | out |
| input topics | per input type, see [`inputs`](configuration.md#inputs) | in |

Message stamps use simulation time, which starts at 0. Set `use_sim_time` to `true` in the nodes that read them.

The node itself runs on wall-clock time. It steps one frame each `1 / (frame_rate × real_time_factor)` s. If it cannot keep this rate, it logs a warning every 5 s.

## Services

| Service | Type | Action |
|---|---|---|
| `/newtonian/pause` | `std_srvs/SetBool` | `true` pauses, `false` resumes |
| `/newtonian/reset` | `std_srvs/Trigger` | returns the scene to its start state |
| `/newtonian/rebuild` | `std_srvs/Trigger` | builds the scene again from the current parameters |

```zsh
ros2 service call /newtonian/pause std_srvs/srv/SetBool "{data: true}"
ros2 service call /newtonian/reset std_srvs/srv/Trigger
```

A reset sets the time to 0, the robot to its start pose and joint positions, and the drive targets to their start values. It clears the last input commands, and turns off lazy drives again.

## Change the scene at runtime

1. Change one or more parameters.
2. Call `rebuild`.

```zsh
ros2 param set /newtonian sim.substeps 8
ros2 service call /newtonian/rebuild std_srvs/srv/Trigger
```

To load many changes, put them in a parameter file and use `ros2 param load /newtonian <file>`.

A rebuild makes a new scene: time starts at 0 again, and topics are created again. If the rebuild fails, the old scene continues to run, and the service response gives the error.

## Viewer

With `sim.viewer: "gl"`, the node opens an OpenGL window. When you close the window, the node stops.

**Control panel** (right side of the window):

| Button | Action |
|---|---|
| Pause / Resume | stops or starts stepping |
| Step | one frame, when paused |
| Reset | same as the `reset` service |

**Mouse and keyboard:**

| Input | Action |
|---|---|
| left drag | look around |
| middle drag | orbit the camera |
| Shift + middle drag | pan the camera |
| Ctrl + middle drag | move the camera nearer or farther |
| scroll | move the camera nearer or farther |
| Ctrl + scroll | change the field of view |
| right drag on a body | pull the body |
| W A S D, arrow keys | move the camera |
| Q, E | move the camera down, up |
| Space | pause or resume |
| `.` | one frame, when paused |
| F | point the camera at the model |
| H | show or hide the panels |
| Esc | close the window |
