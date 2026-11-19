# Benchmarks

- [File structure](#file-structure)
- [`rtf`, `trial`, `seeds`](#rtf-trial-seeds)
- [`robot`](#robot)
- [`limits` and `controllers`](#limits-and-controllers)
- [`axes`](#axes)
- [`suites`](#suites)
- [Thresholds and pass rule](#thresholds-and-pass-rule)
- [Example controller](#example-controller)

## File structure

The config file `config/benchmarks.yaml` of `newtonian_benchmarks` is a parameter file for the node `benchmark`. It holds a whole run : the robot with its simulator scene, the controller, the factors to vary, and the suites that cross them.

```yaml
benchmark:
  ros__parameters:
    rtf: 1.0              # speed of the run
    trial: {...}          # defaults of every trial
    seeds: [0, 1]
    robot: {...}          # everything that depends on the robot
    limits: {...}         # envelope of the commands
    controllers: [...]    # the controllers under test
    axes: {...}           # library of factors
    suites: {...}         # suites that cross the factors
```

Run it with `ros2 run newtonian_benchmarks bench --out <folder>`. `--config <name or path>` selects another file, `--only a,b` runs some suites, `--only-controller a,b` runs some controllers, `--limit N` runs the first N trials, and `--domain` sets the ROS domain (default 60).

The sweep part (`trial`, `axes`, `suites`) does not name a robot, a scene, or a topic. Velocities are fractions of the limits of the controller, pushes are multiples of the body weight, and terrain sizes are multiples of the standing height of the robot. The same sweep runs for another robot or controller by changing the blocks `robot` and `controllers`.

## `rtf`, `trial`, `seeds`

| Key | Description |
|---|---|
| `rtf` | real-time factor of the run. `1.0` is real time. `auto` finds the highest factor at which the simulator and the controller keep up, among `rtf_candidates` (default `[1.0, 1.25, 1.5, 2.0, 3.0, 4.0]`) |
| `trial` | defaults of every trial. `duration` (s of sim time), `settle` (s of standing before the command), `profile` (`step`, `ramp`, `sine`, `stop_and_go`), and the keys of the profile : `ramp` (s), `freq` (Hz), `go` (s). `height` comes from the robot |
| `seeds` | one trial for each point and seed. The seed sets the random generator of the simulator (jitter, drop, sensor noise) |

## `robot`

| Key | Description |
|---|---|
| `name` | name of the robot |
| `height` | nominal base height in m. Default command height, and the unit of the terrain keys that end in `_rel` |
| `topics` | `ground_truth` (odometry without noise, read by the benchmark only), `joint_state`, `joint_command` |
| `defaults` | `thresholds` and `pass` of the suites. A suite can replace single keys |
| `scene` | the simulator parameters : a newtonian parameter file without its `newtonian: ros__parameters:` wrapper, or a string with the path of such a file |

The scene needs a sensor that publishes the ground-truth topic, and drives that are not lazy, so that the robot stands at the start of each trial. The mass of the robot is read from the simulator.

## `limits` and `controllers`

`limits` is the claimed envelope `vx`, `vy`, `wz` as [min, max]. The axes `frac_of` and `from_limits` use it. It belongs to the run, not to a controller, so every controller gets the same commands and the same trial ids. Without `limits`, the limits of the first controller are used.

`controllers` is a list. Each entry is one controller, and the run does every trial with each of them in turn, with one simulator for all of them in a scene. A file can also have one `controller:` block instead of the list. The names must be different.

| Key | Description |
|---|---|
| `name` | name of the controller. Folder of its traces, and column `controller` of the results |
| `command.topic` | where the benchmark publishes `newtonian_msgs/BaseCommand` (vx and vy in the heading frame, yaw rate, height) |
| `launch.cmd` | command line of the controller. `${VAR}` is expanded from the environment |
| `rate_hz` | expected rate of the joint commands. Fewer commands count as missed cycles |
| `reset` | `none` (the controller keeps running when the simulator resets) or `restart` (new process for each trial) |
| `startup_timeout` | s until the first joint command (default 90) |

The controller :

1. subscribes to the sensor topics of the scene and to the base command ;
2. publishes `newtonian_msgs/JointCommand` on the joint command topic ;
3. uses sim time (`use_sim_time:=true`), never the wall clock ;
4. survives a reset of the simulator, where the clock goes back to 0.

The config file of the package has one controller, the example of the package (see [Example controller](#example-controller)), so it runs with nothing else installed.

With more than one controller, the run writes `report_<name>.md` for each of them and `compare.md` with the controllers side by side. `bench --compare <folder> <folder> ... --out <folder>` does the same for runs made apart ; the runs must use the same config except for the controllers. With `rtf: auto`, the factor is found for each controller.

## `axes`

`axes` is a library of factors. Each entry is a list of values, or one of these forms :

| Form | Values |
|---|---|
| `{from_limits: vx, n: 5, scale: 1.0}` | `n` values over the limits of the controller, times `scale` |
| `{frac_of: vx, values: [0.3, 0.6]}` | fractions of the upper limit of the controller |

| Axis | Values |
|---|---|
| `vx`, `vy`, `wz`, `height` | command |
| `profile` | `step`, `ramp`, `sine`, `stop_and_go` |
| `terrain` | `flat` or `{kind: noise\|slope\|stairs\|step, ...}`. Keys that end in `_rel` (`amplitude_rel`, `step_height_rel`) are multiples of the nominal height of the robot |
| `friction` | friction coefficient of all shapes |
| `payload`, `payload_frac` | kg added to the base, or fraction of the mass of the robot |
| `push` | `none` or `{dir: ±x\|±y\|±z\|rx\|ry\|rz, force_bw: multiples of the body weight, duration: s, at: s after the settle time}`. `force` (N or N·m) replaces `force_bw`. A list gives several pushes. For a rotation, `force_bw` is multiplied by the nominal height |
| `cmd_latency`, `sensor_latency` | s, in sim time |
| `jitter`, `drop` | uniform jitter (±s) and loss probability of commands and sensors |
| `start` | start state : `xyz`, `rpy` (offsets), `v`, `w`, `joint_std`, `joint_q` |
| `rtf` | real-time factor of the trial |

## `suites`

Each suite crosses the axes it lists in `cross`, and every combination is a trial for each seed. An item of `cross` is the name of an axis, or `{name: values}` to use other values in that suite. `axes` of a suite adds more axes.

| Key | Description |
|---|---|
| `cross` | axes to cross |
| `sampling` | `grid` (default, the full product), `random` (`samples` different points, `sampling_seed`), or `ablation` (one axis at a time around the first value of each) |
| `trial`, `thresholds`, `pass`, `fixed`, `sim` | merged key by key over the top-level defaults. `fixed` sets values that are not varied |
| `seeds` and any other key | replace the default |

Suites that need the same terrain share one simulator run. The file of the package has 9 suites : `smoke` (4 trials), `velocity`, `speed_on_terrain`, `terrain_friction_speed`, `push_on_terrain`, `push_on_friction`, `latency`, `payload`, and `everything_random`. With 2 seeds it is 1140 trials, several hours. Use `--only`.

## Thresholds and pass rule

`thresholds` set the limits of the metrics. `pass` is the rule of a trial : no fall, every `max_<metric>` respected, and `require_recovery: 1` for the pushes.

| Threshold | Default | Description |
|---|---|---|
| `fall_tilt` | 1.0 rad | tilt of the base that counts as a fall |
| `fall_height` | 0.10 m | base height that counts as a fall |
| `rp_limit` | 0.35 rad | roll and pitch limit of the exceedance count |
| `transient` | 1.0 s | start of the profile that the steady-state errors skip |
| `recover_tilt`, `recover_vel`, `recover_wz` | 0.10 rad, 0.15 m/s, 0.30 rad/s | margins above the values before a push |
| `recover_smooth`, `recover_hold`, `recover_window` | 0.3 s, 0.5 s, 4.0 s | smoothing, hold time, and search time after a push |
| `overrun_frac` | 0.10 | share of late frames that the real-time check allows |
| `gap_limit` | 0.1 s | longest pause of the controller commands that the real-time check allows |

## Example controller

`ros2 run newtonian_benchmarks example` is a minimal Python controller for unitree_go2. It holds the stand pose and ignores the base command, so a trial with `vx` above 0 fails the tracking rule. Its parameters are `rate` (Hz, default 100), `kp`, `kd`, and `busy_ms`, which burns wall-clock time in each cycle to test the real-time check. It is the controller of `config/benchmarks.yaml`.

To test another controller, add an entry to `controllers`. This one is the Rust MPPI controller, which reads the paths of its binary and its config from the environment :

```yaml
controllers:
  - name: mppi_rs
    command: {topic: /go2/base_command}
    launch: {cmd: ["${MPPI_BIN}", "--ros-args", "--params-file", "${MPPI_CFG}", "-p", "use_sim_time:=true", "-r", "__node:=quadruped_mppi"]}
    rate_hz: 500
    startup_timeout: 90
```

Then `--only-controller` selects one of them, and `bench` with no selection runs all of them and writes `compare.md`.
