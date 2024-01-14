package frc.robot.subsystems.drive.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ModuleConstants.kWheelCircumference;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.sim.MAXSwerveModuleSim;
import frc.robot.sim.Simulation;
import frc.robot.sim.SimulationContext;

public class SimModuleIO implements ModuleIO {
  /** Hardware simulation for the swerve module. */
  private final MAXSwerveModuleSim sim;

  // PID constants are different in simulation than in real life
  private final PIDController drivePID = new PIDController(12, 0, 0);
  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.8, 2.445);
  private final PIDController turnPID = new PIDController(10, 0, 0);

  private final SwerveModulePosition currentPosition = new SwerveModulePosition();
  private final Simulation update = this::update;

  private double lastVoltage = 0;

  private SwerveModuleState desiredState = new SwerveModuleState();

  public SimModuleIO(MAXSwerveModuleSim sim) {
    this.sim = sim;

    turnPID.enableContinuousInput(0, 2 * Math.PI);
    SimulationContext.getDefault().addPeriodic(update);
  }

  public SimModuleIO() {
    this(new MAXSwerveModuleSim());
  }

  private void update(double timestep) {
    // Set simulation motor voltages based on the commanded inputs to the module
    double driveVolts = drivePID.calculate(getWheelVelocity()) + driveFeedForward.calculate(desiredState.speedMetersPerSecond);
    double turnVolts = turnPID.calculate(getModuleRotation().getRadians());

    lastVoltage = driveVolts;

    sim.setDriveVoltage(driveVolts);
    sim.setTurnVoltage(turnVolts);

    // Write "sensor" values by integrating the simulated velocities over the past timestep
    currentPosition.angle =
        currentPosition.angle.plus(Rotation2d.fromRadians(sim.getTurnVelocity().in(RadiansPerSecond) * timestep));
    currentPosition.distanceMeters += getWheelVelocity() * timestep;
  }

  @Override
  public double getWheelVelocity() {
    return wheelSpeedFromAngular(sim.getWheelVelocity());
  }

  @Override
  public double getWheelDistance() {
    return currentPosition.distanceMeters;
  }

  @Override
  public Rotation2d getModuleRotation() {
    return currentPosition.angle;
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    this.desiredState = state;
    drivePID.setSetpoint(state.speedMetersPerSecond);
    turnPID.setSetpoint(state.angle.getRadians());
  }

  @Override
  public void stop() {
    setDesiredState(new SwerveModuleState(0, getModuleRotation()));
  }

  @Override
  public void resetEncoders() {
    currentPosition.distanceMeters = 0;
  }

  public double getDriveVoltage() {
    return lastVoltage;
  }

  @Override
  public void close() {
    // Stop simulating the mechanism
    SimulationContext.getDefault().removeMechanism(sim);
    SimulationContext.getDefault().removePeriodic(update);
  }

  /**
   * Converts an angular wheel velocity into the corresponding tangential linear speed.
   *
   * @param angularWheelVelocity the angular velocity of the wheel.
   * @return the tangential speed of the wheel in meters per second.
   */
  private double wheelSpeedFromAngular(Measure<Velocity<Angle>> angularWheelVelocity) {
    return angularWheelVelocity.in(RotationsPerSecond) * kWheelCircumference.in(Meters);
  }
}
