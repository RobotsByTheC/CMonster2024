package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.kDriveKinematics;
import static frc.robot.Constants.DriveConstants.kFrontLeftChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.kFrontRightChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.kRearLeftChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.kRearRightChassisAngularOffset;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.sim.Simulation;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.drive.swerve.SimModuleIO;
import frc.robot.subsystems.drive.swerve.SwerveModule;

public class SimSwerveIO implements SwerveIO {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule rearLeft;
  private final SwerveModule rearRight;

  private final Simulation update = this::update;

  // Assume a 0° starting heading
  private Rotation2d heading = Rotation2d.fromDegrees(0);

  public SimSwerveIO() {
    frontLeft = new SwerveModule(new SimModuleIO(), kFrontLeftChassisAngularOffset);
    frontRight = new SwerveModule(new SimModuleIO(), kFrontRightChassisAngularOffset);
    rearLeft = new SwerveModule(new SimModuleIO(), kRearLeftChassisAngularOffset);
    rearRight = new SwerveModule(new SimModuleIO(), kRearRightChassisAngularOffset);

    SimulationContext.getDefault().addPeriodic(update);
  }

  private void update(double timestep) {
    // Compute the current chassis speeds (x, y, ω)
    var speeds = kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState()
    );

    // Update the heading by integrating the angular velocity by the timestep
    // (note: smaller timesteps will give more accurate results)
    heading = heading.plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * timestep));
  }

  @Override
  public SwerveModule frontLeft() {
    return frontLeft;
  }

  @Override
  public SwerveModule frontRight() {
    return frontRight;
  }

  @Override
  public SwerveModule rearLeft() {
    return rearLeft;
  }

  @Override
  public SwerveModule rearRight() {
    return rearRight;
  }

  @Override
  public Rotation2d getHeading() {
    return heading;
  }

  @Override
  public void resetHeading(Rotation2d heading) {
    this.heading = heading;
  }

  @Override
  public void zeroHeading() {
    heading = Rotation2d.fromDegrees(0);
  }

  @Override
  public void close() {
    SwerveIO.super.close();
    SimulationContext.getDefault().removePeriodic(update);
  }
}
