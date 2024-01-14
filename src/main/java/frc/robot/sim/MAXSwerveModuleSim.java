package frc.robot.sim;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.ModuleConstants.kDrivingMotorReduction;
import static frc.robot.Constants.ModuleConstants.kTurningMotorReduction;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulates a MAX Swerve module with a single NEO driving the wheel and a single NEO 550
 * controlling the module's heading.
 */
public class MAXSwerveModuleSim implements MechanismSim {
  // Simulate both the wheel and module azimuth control as simple flywheels
  // Note that this does not perfectly model reality - in particular, friction is not modeled.
  // The wheel simulation has a much higher moment of inertia than the physical wheel in order to
  // account for the inertia of the rest of the robot. (This is a /very/ rough approximation)
  private final FlywheelSim wheelSim =
      new FlywheelSim(DCMotor.getNEO(1), kDrivingMotorReduction, 0.025);
  private final FlywheelSim turnSim =
      new FlywheelSim(DCMotor.getNeo550(1), kTurningMotorReduction, 0.001);

  private final MutableMeasure<Velocity<Angle>> wheelVelocity = MutableMeasure.zero(RadiansPerSecond);
  private final MutableMeasure<Velocity<Angle>> turnVelocity = MutableMeasure.zero(RadiansPerSecond);

  public MAXSwerveModuleSim() {
    SimulationContext.getDefault().addMechanism(this);
  }

  @Override
  public void update(double timestep) {
    // Update the simulations with most recently commanded voltages
    wheelSim.update(timestep);
    turnSim.update(timestep);

    // Cache the computed velocities
    wheelVelocity.mut_replace(wheelSim.getAngularVelocityRadPerSec(), RadiansPerSecond);
    turnVelocity.mut_replace(turnSim.getAngularVelocityRadPerSec(), RadiansPerSecond);
  }

  public void setDriveVoltage(double volts) {
    wheelSim.setInputVoltage(outputVoltage(volts));
  }

  public void setTurnVoltage(double volts) {
    turnSim.setInputVoltage(outputVoltage(volts));
  }

  /**
   * Gets the angular velocity of the wheel.
   */
  public Measure<Velocity<Angle>> getWheelVelocity() {
    return wheelVelocity;
  }

  /**
   * Gets the angular velocity of the azimuth control.
   */
  public Measure<Velocity<Angle>> getTurnVelocity() {
    return turnVelocity;
  }

  @Override
  public double getCurrentDraw() {
    return wheelSim.getCurrentDrawAmps() + turnSim.getCurrentDrawAmps();
  }
}
