// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(4.5);
    public static final Measure<Velocity<Angle>> maxAngularSpeed = RotationsPerSecond.of(1.0);

    public static final Measure<Velocity<Angle>> directionSlewRate = RadiansPerSecond.of(1.2);
    public static final Measure<Velocity<Dimensionless>> magnitudeSlewRate =
        Percent.of(1.8).per(Second);
    public static final Measure<Velocity<Dimensionless>> rotationalSlewRate =
        Percent.of(2.0).per(Second);

    // Chassis configuration
    public static final Measure<Distance> trackWidth = Inches.of(26.5);
    // Distance between centers of right and left wheels on robot
    public static final Measure<Distance> wheelBase = Inches.of(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase.divide(2), trackWidth.divide(2)),
            new Translation2d(wheelBase.divide(2), trackWidth.divide(-2)),
            new Translation2d(wheelBase.divide(-2), trackWidth.divide(2)),
            new Translation2d(wheelBase.divide(-2), trackWidth.divide(-2)));

    // Angular offsets here describe how the swerve modules are physically rotated with respect to
    // to the chassis. There should be offsets at 0, 90, 180, and 270 degrees for a rectangular
    // chassis.
    public static final Rotation2d frontLeftChassisAngularOffset = Rotation2d.fromDegrees(-90);
    public static final Rotation2d frontRightChassisAngularOffset = Rotation2d.fromDegrees(0);
    public static final Rotation2d rearLeftChassisAngularOffset = Rotation2d.fromDegrees(180);
    public static final Rotation2d rearRightChassisAngularOffset = Rotation2d.fromDegrees(90);

    // SPARK MAX CAN IDs
    public static final int frontLeftDrivingCanId = 10;
    public static final int rearLeftDrivingCanId = 4;
    public static final int frontRightDrivingCanId = 14;
    public static final int rearRightDrivingCanId = 8;

    public static final int frontLeftTurningCanId = 12;
    public static final int rearLeftTurningCanId = 3;
    public static final int frontRightTurningCanId = 7;
    public static final int rearRightTurningCanId = 25;

    public static final boolean gyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int drivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean turningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final Measure<Distance> wheelDiameter = Inches.of(3);
    public static final Measure<Distance> wheelCircumference = wheelDiameter.times(Math.PI);
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
    // Gear ratio of the turning (aka "azimuth") motor. ~46.42:1
    public static final double turningMotorReduction = 9424.0 / 203;
    public static final Measure<Velocity<Distance>> driveWheelFreeSpeed =
        wheelCircumference
            .times(NeoMotorConstants.freeSpeedRpm.in(RotationsPerSecond))
            .divide(drivingMotorReduction)
            .per(Second);

    public static final Measure<Distance> drivingEncoderPositionFactor =
        wheelDiameter.times(Math.PI / drivingMotorReduction);
    public static final Measure<Velocity<Distance>> drivingEncoderVelocityFactor =
        wheelDiameter.times(Math.PI / drivingMotorReduction).per(Minute);

    public static final Measure<Angle> turningEncoderPositionFactor = Rotations.of(1.0);
    public static final Measure<Velocity<Angle>> turningEncoderVelocityFactor =
        RotationsPerSecond.of(1.0);

    public static final Measure<Angle> turningEncoderPositionPIDMinInput = Radians.of(0.0);
    public static final Measure<Angle> turningEncoderPositionPIDMaxInput =
        turningEncoderPositionFactor;

    public static final double drivingP = 0.04;
    public static final double drivingI = 0;
    public static final double drivingD = 0;
    public static final double drivingFF = 1 / driveWheelFreeSpeed.in(MetersPerSecond);
    public static final double drivingMinOutput = -1;
    public static final double drivingMaxOutput = 1;

    public static final double turningP = 1;
    public static final double turningI = 0;
    public static final double turningD = 0;
    public static final double turningFF = 0;
    public static final double turningMinOutput = -1;
    public static final double turningMaxOutput = 1;

    public static final IdleMode drivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode turningMotorIdleMode = IdleMode.kBrake;

    public static final Measure<Current> drivingMotorCurrentLimit = Amps.of(50);
    public static final Measure<Current> turningMotorCurrentLimit = Amps.of(20);
  }

  public static final class OIConstants {
    public static final int driverControllerPort = 1;
    public static final double driveDeadband = 0.05;
    public static final int leftJoystickPort = 2;
    public static final int rightJoystickPort = 3;
  }

  public static final class AutoConstants {
    public static final double pXController = 4;
    public static final double pYController = 4;
    public static final double pThetaController = 2;
  }

  public static final class NeoMotorConstants {
    public static final Measure<Velocity<Angle>> freeSpeedRpm = RPM.of(5676);
  }

  // shooter constants
  public static final class ShooterConstants {
    public static final int rightShooterCanId = 13;
    public static final int leftShooterCanId = 69;
  }

  // intake constants
  public static final class IntakeConstants {
    public static final int intakeCanId = 140;
    public static final Measure<Velocity<Angle>> intakeRotationalSpeed = RPM.of(1.0);
  }

  public static final class LEDConstants {
    public static final int ledPortNumber = 1; // temp
    public static final int ledLength = 30; // temp
  }
}
