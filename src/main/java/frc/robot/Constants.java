// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
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
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final Measure<Velocity<Distance>> kMaxSpeed =
        MetersPerSecond.of(4.5);
    public static final Measure<Velocity<Angle>> kMaxAngularSpeed =
        RotationsPerSecond.of(1.0);

    public static final Measure<Velocity<Angle>> kDirectionSlewRate =
        RadiansPerSecond.of(1.2);
    public static final Measure<Velocity<Dimensionless>> kMagnitudeSlewRate =
        Percent.of(1.8).per(Second);
    public static final Measure<Velocity<Dimensionless>> kRotationalSlewRate =
        Percent.of(2.0).per(Second);

    // Chassis configuration
    public static final Measure<Distance> kTrackWidth = Inches.of(26.5);
    // Distance between centers of right and left wheels on robot
    public static final Measure<Distance> kWheelBase = Inches.of(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase.divide(2), kTrackWidth.divide(2)),
        new Translation2d(kWheelBase.divide(2), kTrackWidth.divide(-2)),
        new Translation2d(kWheelBase.divide(-2), kTrackWidth.divide(2)),
        new Translation2d(kWheelBase.divide(-2), kTrackWidth.divide(-2)));

    // Angular offsets here describe how the swerve modules are physically rotated with respect to
    // to the chassis. There should be offsets at 0, 90, 180, and 270 degrees for a rectangular
    // chassis.
    public static final Rotation2d kFrontLeftChassisAngularOffset = Rotation2d.fromDegrees(-90);
    public static final Rotation2d kFrontRightChassisAngularOffset = Rotation2d.fromDegrees(0);
    public static final Rotation2d kRearLeftChassisAngularOffset = Rotation2d.fromDegrees(180);
    public static final Rotation2d kRearRightChassisAngularOffset = Rotation2d.fromDegrees(90);

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 11;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 10;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 5;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final Measure<Distance> kWheelDiameter = Inches.of(3);
    public static final Measure<Distance> kWheelCircumference = kWheelDiameter.times(Math.PI);
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    // Gear ratio of the turning (aka "azimuth") motor. ~46.42:1
    public static final double kTurningMotorReduction = 9424.0 / 203;
    public static final Measure<Velocity<Distance>> kDriveWheelFreeSpeed =
        kWheelCircumference.times(NeoMotorConstants.kFreeSpeedRpm.in(RotationsPerSecond)).divide(kDrivingMotorReduction).per(Second);

    public static final Measure<Distance> kDrivingEncoderPositionFactor =
        kWheelDiameter.times(Math.PI / kDrivingMotorReduction);
    public static final Measure<Velocity<Distance>> kDrivingEncoderVelocityFactor =
        kWheelDiameter.times(Math.PI / kDrivingMotorReduction).per(Second);

    public static final Measure<Angle> kTurningEncoderPositionFactor =
        Rotations.of(1.0);
    public static final Measure<Velocity<Angle>> kTurningEncoderVelocityFactor =
        RotationsPerSecond.of(1.0);

    public static final Measure<Angle> kTurningEncoderPositionPIDMinInput = Radians.of(0.0);
    public static final Measure<Angle> kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeed.in(MetersPerSecond);
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final Measure<Current> kDrivingMotorCurrentLimit = Amps.of(50);
    public static final Measure<Current> kTurningMotorCurrentLimit = Amps.of(20);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kPXController = 4;
    public static final double kPYController = 4;
    public static final double kPThetaController = 2;
  }

  public static final class NeoMotorConstants {
    public static final Measure<Velocity<Angle>> kFreeSpeedRpm = RPM.of(5676);
  }
}
