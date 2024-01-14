// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  private final SwerveIO io;

  // Odometry class for tracking robot pose
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field = new Field2d();

  public enum ReferenceFrame {
    ROBOT,
    FIELD
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(SwerveIO io) {
    this.io = io;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.driveKinematics,
            Rotation2d.fromDegrees(0),
            io.getModulePositions(),
            new Pose2d(Feet.zero(), Feet.zero(), new Rotation2d(Degrees.zero())),
            // Use default standard deviations of ±4" and ±6° for odometry-derived position data
            // (i.e. 86% of results will be within 4" and 6° of the true value, and 95% will be
            // within
            // ±8" and ±12°)
            VecBuilder.fill(
                Inches.of(4).in(Meters), Inches.of(4).in(Meters), Degrees.of(6).in(Radians)),
            // Use default standard deviations of ±35" and ±52° for vision-system derived position
            // data
            VecBuilder.fill(
                Inches.of(35).in(Meters), Inches.of(35).in(Meters), Degrees.of(52).in(Radians)));

    SmartDashboard.putData("Field", field);

    SmartDashboard.putData("FL", io.frontLeft());
    SmartDashboard.putData("FR", io.frontRight());
    SmartDashboard.putData("RL", io.rearLeft());
    SmartDashboard.putData("RR", io.rearRight());

    SmartDashboard.putNumber("Linear P", 1);
    SmartDashboard.putNumber("Angular P", 1);
  }

  private Pose2d lastPose = new Pose2d();
  private double lastVelocity = 0;

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    poseEstimator.update(io.getHeading(), io.getModulePositions());
    field.setRobotPose(poseEstimator.getEstimatedPosition());

    // Track gyro reading vs odometry - useful for debugging
    SmartDashboard.putNumber("Heading (Gyro)", getHeading().getDegrees());
    SmartDashboard.putNumber("Heading (Odometry)", getPose().getRotation().getDegrees());

    // Track absolute velocity and acceleration. Assumes a nominal 20ms update rate.
    double velocity = lastPose.getTranslation().getDistance(getPose().getTranslation()) / 0.020;
    SmartDashboard.putNumber("Linear Velocity (mps)", velocity);
    SmartDashboard.putNumber(
        "Linear Acceleration (mps^2)", Math.abs(velocity - lastVelocity) / 0.020);

    lastPose = getPose();
    lastVelocity = velocity;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    io.resetHeading(pose.getRotation());
    poseEstimator.resetPosition(io.getHeading(), io.getModulePositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward)
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param orientation What reference frame the speeds are in
   */
  public void drive(
      Measure<Velocity<Distance>> xSpeed,
      Measure<Velocity<Distance>> ySpeed,
      Measure<Velocity<Angle>> rot,
      ReferenceFrame orientation) {
    var speeds =
        switch (orientation) {
          case ROBOT -> new ChassisSpeeds(xSpeed, ySpeed, rot);
          case FIELD -> ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, io.getHeading());
        };
    drive(speeds);
  }

  /**
   * Drives the robot with the given chassis speeds. Note that chassis speeds are relative to the
   * robot's reference frame.
   *
   * @param speeds the speeds at which the robot should move
   */
  public void drive(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.driveKinematics.toSwerveModuleStates(speeds);

    io.setDesiredModuleStates(swerveModuleStates);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    io.setDesiredModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    io.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    io.zeroHeading();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return io.getHeading();
  }

  public SwerveModulePosition[] getModulePositions() {
    return io.getModulePositions();
  }

  public SwerveModuleState[] getModuleStates() {
    return io.getModuleStates();
  }

  public Command followChoreoTrajectory(String trajectoryName) {
    var traj = Choreo.getTrajectory(trajectoryName);

    if (traj == null) {
      // Couldn't find that trajectory, return a command that does nothing
      DriverStation.reportError(
          "Could not find trajectory "
              + trajectoryName
              + ". Make sure the file exists in the src/main/deploy/choreo/ directory and "
              + "the file name is spelled correctly.",
          false);
      return new InstantCommand(() -> {});
    }

    Command followTrajectory =
        Choreo.choreoSwerveCommand(
            traj,
            this::getPose,
            new PIDController(Constants.AutoConstants.pXController, 0, 0),
            new PIDController(Constants.AutoConstants.pYController, 0, 0),
            new PIDController(Constants.AutoConstants.pThetaController, 0, 0),
            this::drive,
            () -> false,
            this);
    followTrajectory.setName(trajectoryName);

    return runOnce(
            () -> {
              // Reset gyro heading and internal odometry to the starting pose of the trajectory.
              // This assumes the robot is already more-or-less at this pose!
              resetOdometry(traj.getInitialPose());
            })
        .andThen(followTrajectory);
  }

  public Command setXCommand() {
    return run(this::setX)
        .until(
            () -> {
              return Arrays.stream(getModuleStates())
                  .allMatch(
                      state -> {
                        double v = Math.abs(state.speedMetersPerSecond);
                        double angle = state.angle.getDegrees();
                        return v <= 0.01 && Math.abs(angle % 45) <= 1;
                      });
            })
        .withName("Set X");
  }

  /**
   * Creates a command that uses joystick inputs to drive the robot. The speeds are field-relative.
   *
   * @param x a supplier for the relative speed that the robot should move on the field's X-axis, as
   *     a number from -1 (towards the alliance all) to +1 (towards the opposing alliance wall).
   * @param y a supplier for the relative speed that the robot should move on the field's Y-axis, as
   *     a number from -1 (towards the right side of the field) to +1 (towards the left side of the
   *     field).
   * @param omega a supplier for the relative speed that the robot should spin about its own axis,
   *     as a number from -1 (maximum clockwise speed) to +1 (maximum counter-clockwise speed).
   * @return the driving command
   */
  public Command driveWithJoysticks(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    var xSpeed = MutableMeasure.zero(MetersPerSecond);
    var ySpeed = MutableMeasure.zero(MetersPerSecond);
    var omegaSpeed = MutableMeasure.zero(RadiansPerSecond);

    return run(() -> {
          xSpeed.mut_setMagnitude(x.getAsDouble() * DriveConstants.maxSpeed.in(MetersPerSecond));
          ySpeed.mut_setMagnitude(y.getAsDouble() * DriveConstants.maxSpeed.in(MetersPerSecond));
          omegaSpeed.mut_setMagnitude(
              omega.getAsDouble() * DriveConstants.maxAngularSpeed.in(RadiansPerSecond));

          drive(xSpeed, ySpeed, omegaSpeed, ReferenceFrame.FIELD);
        })
        .finallyDo(this::setX)
        .withName("Drive With Joysticks");
  }

  @Override
  public void close() {
    io.close();
  }
}
