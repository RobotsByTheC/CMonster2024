package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.kFrontLeftChassisAngularOffset;
import static frc.robot.Constants.DriveConstants.kFrontLeftDrivingCanId;
import static frc.robot.Constants.DriveConstants.kFrontLeftTurningCanId;
import static frc.robot.Constants.DriveConstants.kFrontRightDrivingCanId;
import static frc.robot.Constants.DriveConstants.kFrontRightTurningCanId;
import static frc.robot.Constants.DriveConstants.kRearLeftDrivingCanId;
import static frc.robot.Constants.DriveConstants.kRearLeftTurningCanId;
import static frc.robot.Constants.DriveConstants.kRearRightDrivingCanId;
import static frc.robot.Constants.DriveConstants.kRearRightTurningCanId;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.subsystems.drive.swerve.SwerveModule;
import frc.robot.subsystems.drive.swerve.MAXSwerveModuleIO;

/**
 * IO for a swerve drive using REV MAXSwerve modules driven by a NEO motor and turned by a NEO 550
 * motor.
 */
public class MAXSwerveIO implements SwerveIO {
  private final SwerveModule frontLeft = new SwerveModule(
      new MAXSwerveModuleIO(
          new CANSparkMax(kFrontLeftDrivingCanId, MotorType.kBrushless),
          new CANSparkMax(kFrontLeftTurningCanId, MotorType.kBrushless)
      ),
      kFrontLeftChassisAngularOffset
  );
  private final SwerveModule frontRight = new SwerveModule(
      new MAXSwerveModuleIO(
          new CANSparkMax(kFrontRightDrivingCanId, MotorType.kBrushless),
          new CANSparkMax(kFrontRightTurningCanId, MotorType.kBrushless)
      ),
      kFrontLeftChassisAngularOffset
  );
  private final SwerveModule rearLeft = new SwerveModule(
      new MAXSwerveModuleIO(
          new CANSparkMax(kRearLeftDrivingCanId, MotorType.kBrushless),
          new CANSparkMax(kRearLeftTurningCanId, MotorType.kBrushless)
      ),
      kFrontLeftChassisAngularOffset
  );
  private final SwerveModule rearRight = new SwerveModule(
      new MAXSwerveModuleIO(
          new CANSparkMax(kRearRightDrivingCanId, MotorType.kBrushless),
          new CANSparkMax(kRearRightTurningCanId, MotorType.kBrushless)
      ),
      kFrontLeftChassisAngularOffset
  );

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
