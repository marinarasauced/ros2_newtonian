# Configuration

- [File structure](#file-structure)
- [`sim`](#sim)
- [`solver`](#solver)
- [`environments`](#environments)
- [`robots`](#robots)
- [`sensors`](#sensors)
- [`inputs`](#inputs)

## File structure

The parameter file is a ROS 2 parameter file for the node `newtonian`. It has six groups:

```yaml
newtonian:
  ros__parameters:
    sim: {...}            # timing, device, viewer
    solver: {...}         # physics solver
    environments: {...}   # named entries
    robots: {...}         # named entries
    sensors: {...}        # named entries
    inputs: {...}         # named entries
```

`config/racecar.yaml` is a full example.

**Named entries.** Each entry in `environments`, `robots`, `sensors`, and `inputs` has a name and a `type`:

```yaml
sensors:
  imu: {type: "imu", site: "imu", rate: 200.0}
```

Entries are built in file order. To set a different order, add a `names` list to the group, for example `names: ["scan", "imu"]`. Then only the entries in the list are built.

**Paths.** Keys that take a file (`robots.<name>.file`, `environments.<name>.map`) accept:

| Form | Example |
|---|---|
| package path | `package://newtonian/descriptions/racecar/racecar.xml` |
| Newton asset | `newton://<asset name>` |
| file path | `/home/me/robots/arm.urdf`, `~/robots/arm.urdf` |

**Units and frames.** SI units (m, rad, s, kg, N). The world is z-up. Poses are `[x, y, z, roll, pitch, yaw]`.

**ROS parameter rules.** ROS 2 reads the file before this package does. These errors stop the node before the scene is built:

| Not allowed | Example | Use |
|---|---|---|
| mixed types in a list | `[0, -3.0, 0.0]` | `[0.0, -3.0, 0.0]` |
| empty map | `loader_args: {}` | remove the key |
| empty list | `joints: []` | remove the key |

A list of lists is not an error, but ROS makes it one flat list.

**Package errors.** A bad value stops the build with a `ParameterError` that names the key:

```
sensors.imu.qos expected one of ['default', 'sensor_data'] but got 'fast'
```

> [!IMPORTANT]
> Unknown keys are an error only at the top level and in `sim`. Inside an entry, an unknown key is ignored. Check the spelling of keys in entries.

## `sim`

| Key | Default | Description |
|---|---|---|
| `device` | Warp default | `cpu` or `cuda:0` |
| `viewer` | `none` | `gl` (OpenGL window) or `none` |
| `viewer_thread` | `false` | draw in a thread of its own, so that drawing does not take time from stepping. The window, the OpenGL context and the events then belong to that thread. The simulator hands over a copy of the state for each frame, and skips a frame while the viewer is still drawing the last one. The Reset button of the panel is carried out by the simulator thread. Needs a restart to change (a rebuild keeps the viewer). At the start it wraps the `ScopedStream` blocks of Warp in one lock, because Warp keeps the current stream of a device as one value for all threads |
| `viewer_rate` | `60.0` | frames per wall-clock second (Hz) that the viewer draws, at most. With a `frame_rate` above this value, the viewer draws only some of the frames ; the clock, the sensors and the physics are not changed |
| `frame_rate` | `60.0` | simulation frames per second (Hz). Each frame steps the physics, publishes `/clock` and the sensors |
| `substeps` | `2` | physics steps per frame (≥ 1) |
| `real_time_factor` | `1.0` | simulated seconds per wall-clock second |
| `cuda_graph` | `true` | capture one frame as a CUDA graph (CUDA only) |
| `gravity` | `-9.81` | m/s², along z |
| `seed` | `42` | seed for sensor noise |

The physics time step is `1 / (frame_rate × substeps)`.

## `solver`

```yaml
solver:
  type: "mujoco"          # 'featherstone' | 'kamino' | 'mujoco' | 'xpbd'
  mujoco:                 # arguments for the Newton solver of this type
    iterations: 50
  collision_args:         # optional; arguments for Newton's collision pipeline
    ...
```

Only the block that has the name of `type` is used. Its keys go to the Newton solver without change. See the Newton documentation for the keys of each solver.

**Contacts.** `collision_args` is used only when Newton's collision pipeline makes the contacts:

| Solver | Contacts from | Change with |
|---|---|---|
| `mujoco` | MuJoCo | `use_mujoco_contacts: false` uses Newton's pipeline |
| `kamino` | Newton's pipeline | `use_collision_detector: true` uses Kamino's own detector |
| `featherstone`, `xpbd` | Newton's pipeline | |

**Capabilities.** A scene that needs a capability the solver does not have stops the build with an error.

| Solver | Contact sensors | Joint effort in `jointstate` | Lazy drives |
|---|---|---|---|
| `mujoco` | yes | yes | yes |
| `featherstone` | yes | no | yes |
| `kamino` | yes, with Newton's pipeline only | no | no |
| `xpbd` | yes | no | yes |

**Results with the racecar:**

| Solver | Result | Notes |
|---|---|---|
| `mujoco` | recommended | fast |
| `featherstone` | works | needs `body_armature: 0.01` at 4 substeps, or about 64 substeps without it |
| `kamino` | works, slow | real time on a ground plane; about 5 fps with the track walls |
| `xpbd` | does not work | the joint drives do not turn the wheels |

## `environments`

More than one environment is allowed.

| Type | Keys |
|---|---|
| `groundplane` | `height` (m, default `0.0`) |
| `track` | `map`: a ROS `map_server` YAML file; `wall_height` (m, default `0.3`) |

All types also accept:

| Key | Description |
|---|---|
| `color` | `[r, g, b]`, 0 to 1. Default: `[0.8, 0.8, 0.8]` (white) for `groundplane`, `[0.6, 0.6, 0.6]` (grey) for `track` |
| `mu`, `restitution`, `ke`, `kd`, `kf`, `margin` | contact settings (Newton shape config) |

**Track map.** The map YAML file uses these keys:

| Key | Description |
|---|---|
| `image` | image file, relative to the YAML file |
| `resolution` | m per pixel |
| `origin` | `[x, y, yaw]` of the bottom-left pixel |
| `negate` | `1` inverts the image (default `0`) |
| `occupied_thresh` | occupancy above this value is a wall (default `0.65`) |

Dark pixels are occupied. Each block of occupied pixels becomes a box wall of `wall_height`. All walls have the same height and color. Gray and white pixels are free, and any dark pixel is a wall, also one outside the corridor.

The package has two racecar maps, in `descriptions/racecar/tracks/<name>/<name>.yaml`:

| Map | Shape |
|---|---|
| `oval` | oval, 20 × 10 m |
| `poire` | pear with a stem and a leaf, 16 × 21.5 m |

A track is two wall outlines, one outer and one inner, on a free background. The outlines are 2 pixels (0.1 m at `resolution` 0.05) thick, and the corridor between them is 2 m wide. The stem and the leaf of `poire` are outlines of the same thickness outside the corridor. They are walls, and do not change the driving.

## `robots`

One robot per scene.

| Key | Default | Description |
|---|---|---|
| `type` | required | `mjcf`, `urdf`, or `usd` |
| `file` | required | path to the robot file, see [Paths](#file-structure) |
| `pose` | origin | `[x, y, z, roll, pitch, yaw]` |
| `floating` | `true` | floating base |
| `body_armature` | `0.0` | inertia added to each body (kg·m²) |
| `loader_args` | | arguments for Newton's `add_mjcf`, `add_urdf`, or `add_usd` |
| `joint_cfg`, `shape_cfg` | | fields of Newton's default joint and shape config |
| `drives` | | named groups of joints, see below |
| `colors` | | named groups `{shapes: <glob>, color: [r, g, b]}`; colors in the robot file stay for other shapes |
| `initial_joint_q` | | `<joint glob>: <value>` start positions, also the start targets |

**Names.** Globs and names in all keys use the short names from the robot file (`front_left_wheel_joint`), not the full Newton labels.

**Drives.** Each group selects joints with a glob and sets their drive. Groups are applied in file order. A later group overwrites the keys it sets.

```yaml
drives:
  wheels: {joints: "*_wheel_joint", mode: "velocity", kd: 2.0, effort_limit: 5.0}
```

| Key | Description |
|---|---|
| `joints` | glob, or a list of globs |
| `mode` | `none`, `position`, `velocity`, `position_velocity`, `effort` |
| `ke`, `kd` | stiffness and damping of the drive |
| `effort_limit`, `velocity_limit` | limits |
| `armature`, `damping`, `friction` | joint properties |
| `lazy` | `true`: the drive has no force until the first input message arrives. After a reset, it has no force again |

## `sensors`

**Common keys:**

| Key | Default | Description |
|---|---|---|
| `topic` | entry name | ROS topic |
| `rate` | `0.0` | Hz; `0` means each frame |
| `qos` | `sensor_data`; `default` for `jointstate` | `sensor_data` (best effort) or `default` (reliable) |
| `qos_depth` | `10` | history depth for `default` |
| `frame_id` | see below | header frame |
| `robot` | the only robot | robot name |

A sensor updates once per frame at most. A `rate` above `sim.frame_rate` has the same result as `0`.

**Mount.** The `imu`, `lidar`, `camera_color`, and `camera_depth` sensors attach to the robot in one of two ways:

- `site: <site name>`: a site in the robot file. Default `frame_id`: the site name.
- `body: <body name>` and `offset: [x, y, z, roll, pitch, yaw]`. Default `frame_id`: the entry name.

Cameras look along +x of the mount frame. The lidar scans in the xy plane of the mount frame, with angle 0 on +x.

**Types:**

| Type | Message | Keys (default) |
|---|---|---|
| `imu` | `sensor_msgs/Imu` | `accel_noise_std`, `gyro_noise_std` (0); `orientation` (`true`) |
| `lidar` | `sensor_msgs/LaserScan` | `samples` (1080); `angle_min`, `angle_max` (±2.356 rad); `range_min` (0.06 m); `range_max` (30 m); `noise_std` (0) |
| `camera_color` | `sensor_msgs/Image`, `rgb8` | `width` (640); `height` (480); `fov` (vertical, 1.0 rad); `max_distance` (1000 m); `camera_info` (`true`) |
| `camera_depth` | `sensor_msgs/Image`, `32FC1` in m | same as `camera_color` |
| `odometry` | `nav_msgs/Odometry` | `body` (root body); `child_frame_id` (body name); `pose_noise_std`, `twist_noise_std` (0); `frame_id` (`odom`) |
| `jointstate` | `sensor_msgs/JointState` | `joints` (glob, all revolute and prismatic joints); `frame_id` (empty) |
| `contact_force` | `geometry_msgs/WrenchStamped` | `bodies` (glob, required); `frame_id` (`world`) |
| `contact_scalar` | `geometry_msgs/WrenchStamped` | `bodies` (glob, required); `frame_id` (`world`) |

**Values per type:**

| Type | Values |
|---|---|
| `imu` | orientation in the world frame. With `orientation: false`, `orientation_covariance[0]` is `-1` |
| `lidar` | a range above `range_max` or no hit is `+inf`. A range below `range_min` is `-inf` |
| `camera_depth` | a pixel with no hit is `NaN` |
| `camera_*` | `frame_id` is the mount frame, not an optical frame |
| `odometry` | pose of the body origin in the world frame (`odom` = world origin). Twist in the body frame |
| `jointstate` | `position` and `velocity` of each joint. `effort` only with the `mujoco` solver |
| `contact_force` | net contact force on the selected bodies, world frame |
| `contact_scalar` | magnitude of that force in `force.z` |

Noise is Gaussian with the given standard deviation. `sim.seed` sets the noise sequence.

**Camera info.** A camera also publishes `sensor_msgs/CameraInfo` on `<topic folder>/camera_info` (`cam/image` gives `cam/camera_info`). Set `camera_info: false` to stop it.

## `inputs`

**Common keys:**

| Key | Default | Description |
|---|---|---|
| `topic` | entry name | ROS topic |
| `qos` | `default` | `default` (reliable) or `sensor_data` (best effort) |
| `qos_depth` | `10` | history depth |
| `robot` | the only robot | robot name |
| `timeout` | per type | s; see the types below. `0` keeps the last command |

The first message on a `racecar` or `jointcommand` input turns on the [lazy drives](#robots) of its robot. A `markers` input does not.

**Types:**

| Type | Message | Description |
|---|---|---|
| `racecar` | `geometry_msgs/Twist` | Ackermann steering. `linear.x` is speed (m/s), `angular.z` is yaw rate (rad/s). Default `timeout` 0.5 s; after the timeout, the car stops at `max_decel` and the steering goes to 0 |
| `jointcommand` | `newtonian_msgs/JointCommand` | joint targets and gains by name. `position` and `velocity` set the drive targets; `effort` sets a feed-forward force, which is 0 when it is missing or NaN. `kp` and `kd` set the gains `ke` and `kd` of the drive. Missing or NaN targets and gains, and unknown names, are ignored. Default `timeout` 0; the targets and gains stay after a timeout |
| `markers` | `visualization_msgs/MarkerArray` | draws the `LINE_LIST` and `LINE_STRIP` markers in the viewer, for example the rollouts of a planner. It does not command the robot. Default `timeout` 0.5 s of simulation time; with no new message for this time, the lines are hidden |

**`racecar` keys:**

| Key | Default | Description |
|---|---|---|
| `wheelbase`, `track`, `wheel_radius` | 0.33, 0.24, 0.05 | m |
| `max_speed` | 10.0 | m/s |
| `max_accel`, `max_decel` | 3.0, 6.0 | m/s² |
| `max_steer` | 0.4189 | rad |
| `max_steer_rate` | 3.0 | rad/s |
| `steer_joints` | | steered joints: `{left: <joint>, right: <joint>}` |
| `wheel_joints` | | driven joints: `{front_left, front_right, rear_left, rear_right}` |

Speed changes at `max_accel` or `max_decel`, and steering at `max_steer_rate`.

**`markers` keys:**

| Key | Default | Description |
|---|---|---|
| `line_width` | `2.0` | px; the viewer has one line width for all lines |

Each marker is one batch of lines, named by its `ns` and `id`. A new message replaces the batch with the same `ns` and `id`. A line has the color of its first vertex (`colors`), or `color` when the marker has no `colors`. Other marker types are ignored. Nothing is drawn without `sim.viewer: "gl"`.
