# Development

- [Architecture](#architecture)
- [Build order](#build-order)
- [Step order](#step-order)
- [Add a plugin](#add-a-plugin)
- [Check the code](#check-the-code)

## Architecture

```zsh
newtonian/

    node.py                  ROS 2 node

    modules/
        params.py            Parameters
        registry.py          Registry
        arrays.py
        transforms.py
        paths.py

        physics/
            simulator.py     NewtonianSimulator
            viewer.py        NewtonianViewer

        plugins/
            base.py          Plugin
            environments/    groundplane, track
            robots/          mjcf, urdf, usd
            sensors/         imu, lidar, camera, odometry, jointstate, contact
            inputs/          racecar, jointcommand, markers
            solvers/         mujoco, featherstone, kamino, xpbd

config/                      parameter files
descriptions/                robot files
launch/                      sim.launch.py
```

## Build order

When the simulator builds a scene, it calls:

1. `solver.before_build(builder)`
2. `on_build(sim, builder)` for each plugin: environments, robots, inputs, sensors
3. `solver.before_finalize(builder)`, then the model is finalized
4. `on_model(sim)` for each plugin, in the same order
5. `solver.create(model)`

On each reset, it calls `reset()` for each plugin, in the same order.

## Step order

For each frame, the node:

1. calls `control(sim)` for each input; the inputs write joint targets to `sim.buffers`
2. copies the buffers to the device, if they changed
3. runs `substeps` physics steps (as one CUDA graph, if enabled)
4. publishes `/clock`
5. calls `update(sim, cache)` for each sensor that is due, then publishes `to_msg(stamp)`
6. draws the viewer

## Add a plugin

1. Write a class in `plugins/<kind>/`, based on the class in `plugins/<kind>/base.py`.
2. Register the class with a `type` name.
3. Import the module in `plugins/<kind>/__init__.py`.

| Kind | Base class | Methods to write |
|---|---|---|
| environment | `Environment` | `on_build` |
| robot | `Robot` | `load` |
| sensor | `Sensor`, `SensorFixed` (mounted), `SensorRaytrace` (ray cast) | `update`, `to_msg`; set `msg_type` |
| input | `Input` | `handle`, or `control` for an input that does not command a robot (`markers`); set `msg_type` |
| solver | `Solver` | set `solver_cls`; change `create` and the capability properties if necessary |

Example sensor:

```python
@SENSORS.register("my_sensor")
class SensorMySensor(Sensor):
    msg_type = MyMsg

    def update(self, sim, cache) -> None:
        ...

    def to_msg(self, stamp) -> MyMsg:
        ...
```

**Rules:**

- Read keys with the typed getters of `Parameters` (`get_int`, `get_float`, `get_bool`, `get_choice`). They give a `ParameterError` with the full key name.
- Find robot parts with `self.robot.find_joints`, `find_bodies`, `find_body`, `find_shapes`. They use the short names and give a `ParameterError` when nothing matches.
- In a sensor, read body poses from `cache.body_q()`. Call `cache.bvh_refit_shapes()` before a ray cast. The cache makes sure that each frame does this work only once.
- In an input, write to `sim.buffers` and set `sim.buffers.dirty = True`. Do not write to `sim.control` directly.
- To draw lines in the viewer, call `sim.viewer.draw_lines(name, starts, ends, colors, device, width)`. `sim.viewer` is `None` without a viewer, so check it first.
- Put ROS message imports in the plugin module.

## Check the code

The package has no tests yet. Check types with Pyright:

```zsh
pip install pyright
pyright newtonian
```

Then run the node with a scene that uses the changed plugin. The node adds its own ROS parameters, so a test of the simulator alone is not sufficient.
