# Canonical Signal IDs

Canonical IDs are stable semantic names for `signals[].canonical.id`.
Applications should use them for feature mapping instead of vehicle-specific
channel names.

## Rules

- Use lowercase ASCII.
- Use `.` for semantic hierarchy.
- Use `_` only inside one enum-like value, such as `front_left`.
- Prefer target-first naming: `engine.coolant.temperature`, not
  `temperature.engine_coolant`.
- Do not encode CAN IDs, diagnostic service IDs, vendor names, or transport
  details in the canonical ID.
- Do not assign a canonical ID when the channel meaning is ambiguous.

Pattern:

```text
<domain>.<target>.<quantity>[.<qualifier>]
```

## Registry

Powertrain:

| Canonical ID | Preferred Unit |
| --- | --- |
| `engine.speed` | `rpm` |
| `engine.coolant.temperature` | `C` |
| `engine.coolant.pressure` | `bar` |
| `engine.oil.temperature` | `C` |
| `engine.oil.pressure` | `bar` |
| `engine.oil.level` | `%` |
| `engine.load` | `%` |
| `engine.runtime` | `s` |
| `engine.injection.time` | `ms` |
| `engine.injection.duty` | `%` |
| `engine.ignition.advance` | `deg` |
| `engine.dwell_time` | `ms` |
| `engine.mass_air_flow` | `kg/s` |
| `engine.torque` | `Nm` |
| `engine.state` | `state` |
| `engine.malfunction_indicator` | `bool` |
| `engine.oil.warning_lamp` | `bool` |
| `intake_air.temperature` | `C` |
| `intake_air.boost_pressure` | `bar` |
| `intake_manifold.pressure` | `bar` |
| `exhaust.air_fuel_ratio` | `ratio` |
| `exhaust.lambda` | `lambda` |
| `exhaust.lambda.bank1` | `lambda` |
| `exhaust.lambda.bank2` | `lambda` |
| `exhaust.gas.temperature` | `C` |
| `throttle.position` | `%` |
| `accelerator.pedal.position` | `%` |
| `fuel.level` | `%` |
| `fuel.pressure` | `bar` |
| `fuel.temperature` | `C` |
| `fuel.used` | `L` |
| `fuel.remaining` | `L` |
| `fuel.consumption` | `L/h` |
| `fuel.flow` | `L/h` |

Vehicle and chassis:

| Canonical ID | Preferred Unit |
| --- | --- |
| `vehicle.speed` | `km/h` |
| `vehicle.odometer` | `km` |
| `vehicle.acceleration.longitudinal` | `g` |
| `vehicle.acceleration.lateral` | `g` |
| `vehicle.acceleration.vertical` | `g` |
| `vehicle.yaw_rate` | `deg/s` |
| `vehicle.roll_rate` | `deg/s` |
| `wheel.speed` | `km/h` |
| `wheel.speed.front_left` | `km/h` |
| `wheel.speed.front_right` | `km/h` |
| `wheel.speed.rear_left` | `km/h` |
| `wheel.speed.rear_right` | `km/h` |
| `wheel.slip` | `%` |
| `steering.wheel.angle` | `deg` |
| `steering.wheel.speed` | `deg/s` |
| `steering.wheel.direction` | `state` |
| `brake.pressure` | `bar` |
| `brake.pressure.front` | `bar` |
| `brake.pressure.rear` | `bar` |
| `brake.pedal.position` | `%` |
| `brake.switch` | `bool` |
| `brake.light.switch` | `bool` |
| `tire.pressure.front_left` | `bar` |
| `tire.pressure.front_right` | `bar` |
| `tire.pressure.rear_left` | `bar` |
| `tire.pressure.rear_right` | `bar` |
| `tire.temperature.front_left` | `C` |
| `tire.temperature.front_right` | `C` |
| `tire.temperature.rear_left` | `C` |
| `tire.temperature.rear_right` | `C` |

Drivetrain and electrical:

| Canonical ID | Preferred Unit |
| --- | --- |
| `transmission.gear.selected` | `gear` |
| `transmission.gear.requested` | `gear` |
| `transmission.oil.temperature` | `C` |
| `clutch.position` | `%` |
| `clutch.switch` | `bool` |
| `battery.voltage` | `V` |
| `ecu.board.temperature` | `C` |
| `abs.active` | `bool` |
| `abs.event` | `state` |
| `abs.mode` | `state` |
| `abs.disabled` | `bool` |
| `abs.warning_lamp` | `bool` |
| `abs.fault` | `bool` |
| `traction_control.mode` | `state` |
| `traction_control.active` | `bool` |
| `traction_control.event` | `state` |
| `traction_control.disabled` | `bool` |
| `traction_control.warning_lamp` | `bool` |
| `traction_control.fault` | `bool` |
| `stability_control.mode` | `state` |
| `stability_control.active` | `bool` |
| `stability_control.event` | `state` |
| `stability_control.disabled` | `bool` |
| `stability_control.warning_lamp` | `bool` |
| `stability_control.fault` | `bool` |
| `parking_brake.switch` | `bool` |
| `transmission.neutral.switch` | `bool` |
| `timing.lap_time` | `s` |

Body and lighting:

| Canonical ID | Preferred Unit |
| --- | --- |
| `lighting.headlight` | `bool` |
| `lighting.high_beam` | `bool` |
| `lighting.position` | `bool` |
| `lighting.indicator.left` | `bool` |
| `lighting.indicator.right` | `bool` |
| `door.open.front_left` | `bool` |
| `door.open.front_right` | `bool` |
| `seat_belt.driver.fastened` | `bool` |
| `wiper.trigger` | `bool` |
| `ignition.on` | `bool` |

`battery.voltage` means the low-voltage auxiliary battery. Use
`high_voltage_battery.*` for hybrid and EV traction batteries.

Hybrid and EV traction battery:

| Canonical ID | Preferred Unit |
| --- | --- |
| `high_voltage_battery.state_of_charge` | `%` |
| `high_voltage_battery.state_of_health` | `%` |
| `high_voltage_battery.voltage` | `V` |
| `high_voltage_battery.current` | `A` |
| `high_voltage_battery.power` | `kW` |
| `high_voltage_battery.temperature` | `C` |
| `high_voltage_battery.temperature.min` | `C` |
| `high_voltage_battery.temperature.max` | `C` |

Electric drive:

| Canonical ID | Preferred Unit |
| --- | --- |
| `electric_motor.speed` | `rpm` |
| `electric_motor.speed.front` | `rpm` |
| `electric_motor.speed.rear` | `rpm` |
| `electric_motor.torque` | `Nm` |
| `electric_motor.torque.front` | `Nm` |
| `electric_motor.torque.rear` | `Nm` |
| `electric_motor.power` | `kW` |
| `electric_motor.temperature` | `C` |
| `inverter.temperature` | `C` |
| `inverter.temperature.front` | `C` |
| `inverter.temperature.rear` | `C` |
| `inverter.dc_voltage` | `V` |
| `inverter.phase_current` | `A` |

Charging and conversion:

| Canonical ID | Preferred Unit |
| --- | --- |
| `charging.status` | `state` |
| `charging.power` | `kW` |
| `charging.voltage` | `V` |
| `charging.current` | `A` |
| `charging.plug_connected` | `bool` |
| `dc_dc_converter.output_voltage` | `V` |
| `dc_dc_converter.output_current` | `A` |
| `dc_dc_converter.temperature` | `C` |

Hybrid system:

| Canonical ID | Preferred Unit |
| --- | --- |
| `hybrid.system.mode` | `state` |
| `hybrid.system.power` | `kW` |

Thermal system:

| Canonical ID | Preferred Unit |
| --- | --- |
| `thermal.coolant.temperature` | `C` |

Environment:

| Canonical ID | Preferred Unit |
| --- | --- |
| `ambient_air.temperature` | `C` |
| `ambient_air.pressure` | `bar` |
