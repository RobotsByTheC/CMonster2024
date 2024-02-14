// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements AutoCloseable, Sendable {
  /** The I/O layer used to communicate with the module hardware. */
  private final ModuleIO io;

  /**
   * The angular offset of this module relative to the center of the chassis. An angle of 0 is
   * straight forward, with positive values increasing counter-clockwise.
   */
  private final Rotation2d angularOffset;

  private SwerveModuleState targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

  /**
   * Constructs a swerve module.
   *
   * @param io the IO object to use to interact with the swerve module hardware
   * @param angularOffset the angular
   */
  public SwerveModule(ModuleIO io, Rotation2d angularOffset) {
    this.io = io;
    this.angularOffset = angularOffset;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        io.getWheelVelocity(), io.getModuleRotation().minus(angularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        io.getWheelDistance(), io.getModuleRotation().minus(angularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    var correctedState =
        new SwerveModuleState(
            desiredState.speedMetersPerSecond, desiredState.angle.plus(angularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    var optimizedState = SwerveModuleState.optimize(correctedState, io.getModuleRotation());
    this.targetState = optimizedState;

    io.setDesiredState(optimizedState);
  }

  /** Zeroes all the swerve module encoders. */
  public void resetEncoders() {
    io.resetEncoders();
  }

  /** Immediately stops all motors on the module and halts movement. */
  public void stop() {
    io.stop();
  }

  @Override
  public void close() {
    io.close();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.addDoubleProperty("Speed (mps)", io::getWheelVelocity, null);
    builder.addDoubleProperty("Angle (deg)", () -> io.getModuleRotation().getDegrees(), null);
    builder.addDoubleProperty("Target Speed (mps)", () -> targetState.speedMetersPerSecond, null);
    builder.addDoubleProperty("Target Angle (deg)", () -> targetState.angle.getDegrees(), null);
    builder.addDoubleProperty("Turn Voltage", () -> io.getTurnVoltage(), null);

    if (io instanceof SimModuleIO s) {
      builder.addDoubleProperty("Applied Drive Voltage", s::getDriveVoltage, null);
    }
  }

  public void setVoltageForDrivingMotor(Measure<Voltage> v) {
    io.setDrivingMotorVoltage(v);
  }
}
