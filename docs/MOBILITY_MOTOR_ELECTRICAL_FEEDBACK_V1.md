# Mobility motor electrical feedback v1

`MotorElectricalFeedbackSensor` separates completed motor-plant state from the
electrical values a controller can observe. It never reads an actuator command or
reconstructs a measurement from a target.

## Contract

The motor plant publishes `DcMotorCompletedTelemetry` after a completed electrical
step: realized terminal voltage, realized current, back-EMF, saturation flags,
failure mode, and optional winding temperature. `DcMotorEvaluation::completed_telemetry`
bridges the existing equivalent-circuit result into that explicit source component.

The sensor frontend applies this fixed order:

1. exact schedule and sampling phase;
2. completed telemetry lookup;
3. per-channel calibration offset;
4. seeded, sample-indexed Gaussian white noise;
5. measurement-range clipping with visible saturation flags;
6. ADC quantization;
7. stuck-value substitution or frame dropout;
8. DataBus availability latency.

The payload carries terminal voltage, current, optional winding temperature,
scheduled time, phase error, and saturation/fault status. If no thermal plant is
attached, temperature is `None` and status is `temperature_unavailable`; RNE does
not invent ambient or zero temperature. Plant current limiting and frontend ADC
saturation are both visible.

## Research and OSS correspondence

- ODrive's authoritative interface exposes measured phase and d/q current,
  DC-bus current, configured measurement range, current-limit violations,
  unavailable-current errors, thermistor state, and effective thermally limited
  current separately from setpoints:
  https://github.com/odriverobotics/ODrive/blob/master/Firmware/odrive-interface.yaml
- ODrive's CAN protocol transports measured q-axis current, motor/FET temperature,
  and bus voltage/current as separate controller-to-host messages:
  https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
- ODrive's motor implementation derives shunt-amplifier gain and maximum measurable
  current from the requested range and applies thermal current limits, which is why
  RNE keeps measurement range distinct from plant current limits:
  https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/motor.cpp
- Anuchin et al., *Quick Compensation Method of Motor Phase Current Sensor
  Offsets without Motor Parameters*, motivates explicit current-sensor offset
  calibration rather than folding offset into the motor model:
  https://itohserver01.nagaokaut.ac.jp/itohlab/paper/2017/20171001_ECCE/koroku.pdf

The RNE v1 payload is an equivalent DC or torque-producing current channel, not
three-phase ADC or FOC d/q telemetry. Profiles must state which real signal they
map into `current_a`.

## Known omissions

- no PWM switching, shunt sampling window, phase reconstruction, common-mode
  rejection, anti-alias filter, or current-controller bandwidth;
- no correlated drift, gain error, hysteresis, thermistor time constant, or ADC
  integral/differential nonlinearity;
- no winding thermal plant yet; temperature is observable only when another plant
  supplies completed temperature;
- no bus-voltage sag, inverter loss, phase current, or separate FET temperature;
- completed telemetry is an explicit boundary but is not yet wired through every
  mobile plant scenario.

M3 must drive this frontend from the integrated motor/transmission plant and score
locked-rotor, acceleration, braking, saturation, dropout, and stuck cases. M5 must
identify range, offset, noise, quantization, latency, and thermal behavior from real
controller logs before a hardware-matched profile can be claimed.
