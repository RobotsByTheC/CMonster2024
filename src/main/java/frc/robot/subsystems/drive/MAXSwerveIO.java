package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.frontLeftChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.frontLeftDrivingCanId;
import static frc.robot.Constants.DriveConstants.frontLeftTurningCanId;
import static frc.robot.Constants.DriveConstants.frontRightChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.frontRightDrivingCanId;
import static frc.robot.Constants.DriveConstants.frontRightTurningCanId;
import static frc.robot.Constants.DriveConstants.rearLeftChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.rearLeftDrivingCanId;
import static frc.robot.Constants.DriveConstants.rearLeftTurningCanId;
import static frc.robot.Constants.DriveConstants.rearRightChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.rearRightDrivingCanId;
import static frc.robot.Constants.DriveConstants.rearRightTurningCanId;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.subsystems.drive.swerve.MAXSwerveModuleIO;
import frc.robot.subsystems.drive.swerve.SwerveModule;

/**
 * IO for a swerve drive using REV MAXSwerve modules driven by a NEO motor and turned by a NEO 550
 * motor.
 */
public class MAXSwerveIO implements SwerveIO {
  private final SwerveModule frontLeft =
      new SwerveModule(
          new MAXSwerveModuleIO(
              new CANSparkMax(frontLeftDrivingCanId, MotorType.kBrushless),
              new CANSparkMax(frontLeftTurningCanId, MotorType.kBrushless)),
          frontLeftChassisAngularOffset);
  private final SwerveModule frontRight =
      new SwerveModule(
          new MAXSwerveModuleIO(
              new CANSparkMax(frontRightDrivingCanId, MotorType.kBrushless),
              new CANSparkMax(frontRightTurningCanId, MotorType.kBrushless)),
          frontRightChassisAngularOffset);
  private final SwerveModule rearLeft =
      new SwerveModule(
          new MAXSwerveModuleIO(
              new CANSparkMax(rearLeftDrivingCanId, MotorType.kBrushless),
              new CANSparkMax(rearLeftTurningCanId, MotorType.kBrushless)),
          rearLeftChassisAngularOffset);
  private final SwerveModule rearRight =
      new SwerveModule(
          new MAXSwerveModuleIO(
              new CANSparkMax(rearRightDrivingCanId, MotorType.kBrushless),
              new CANSparkMax(rearRightTurningCanId, MotorType.kBrushless)),
          rearRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

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
    return Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kYaw));
  }

  @Override
  public void resetHeading(Rotation2d heading) {
    gyro.setGyroAngle(IMUAxis.kYaw, heading.getDegrees());
  }

  @Override
  public void zeroHeading() {
    gyro.reset();
  }
}
