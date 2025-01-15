// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim sim;
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private Voltage ffVolts = Volts.of(0.0);
  private Voltage appliedVolts = Volts.of(0.0);

  public FlywheelIOSim() {
    DCMotor gearbox = DCMotor.getNEO(1);
    sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox, 1.5, 0.004), gearbox, 0.02);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      Voltage calculated = Volts.of(pid.calculate(sim.getAngularVelocityRadPerSec()));
      double requested = calculated.plus(ffVolts).in(Volts);
      appliedVolts = Volts.of(
          MathUtil.clamp(requested, -12.0, 12.0)
      );
      sim.setInputVoltage(appliedVolts.in(Volts));
    }

    sim.update(0.02);

    inputs.positionRad = Rotations.of(0);
    inputs.velocityRadPerSec = sim.getAngularVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(Voltage volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts.in(Volts));
  }

  @Override
  public void setVelocity(AngularVelocity velocity, Voltage ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocity.in(RadiansPerSecond));
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(Volts.of(0.0));
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
