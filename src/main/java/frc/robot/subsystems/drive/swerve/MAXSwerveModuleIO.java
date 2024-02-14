package frc.robot.subsystems.drive.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Swerve module IO implemented using REV SparkMaxes, a NEO driving motor, and a NEO 550 turning
 * motor.
 */
public class MAXSwerveModuleIO implements ModuleIO {
  private final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController drivingPIDController;
  private final SparkPIDController turningPIDController;

  public MAXSwerveModuleIO(CANSparkMax drivingSparkMax, CANSparkMax turningSparkMax) {
    this.drivingSparkMax = drivingSparkMax;
    drivingPIDController = drivingSparkMax.getPIDController();
    drivingEncoder = drivingSparkMax.getEncoder();

    this.turningSparkMax = turningSparkMax;
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    turningPIDController = turningSparkMax.getPIDController();

    // Send SPARK MAX configurations to the controllers.
    configureDrivingController();
    configureTurningController();

    // Wait a little bit to ensure all the configuration packets have been received by the motor
    // controllers before sending the command to burn the flash. Otherwise the controllers might
    // not have all the configuration options set when burning the flash.
    Timer.delay(0.100);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    resetEncoders();
  }

  private void configureDrivingController() {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.restoreFactoryDefaults();
    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingPIDController.setFeedbackDevice(drivingEncoder);
    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingEncoder.setPositionConversionFactor(
        Constants.ModuleConstants.drivingEncoderPositionFactor.in(Meters));
    drivingEncoder.setVelocityConversionFactor(
        Constants.ModuleConstants.drivingEncoderVelocityFactor.in(MetersPerSecond));

    // Set the PID gains for the driving motor.
    drivingPIDController.setP(Constants.ModuleConstants.drivingP);
    drivingPIDController.setI(Constants.ModuleConstants.drivingI);
    drivingPIDController.setD(Constants.ModuleConstants.drivingD);
    drivingPIDController.setFF(Constants.ModuleConstants.drivingFF);
    drivingPIDController.setOutputRange(
        Constants.ModuleConstants.drivingMinOutput, Constants.ModuleConstants.drivingMaxOutput);

    drivingSparkMax.setIdleMode(Constants.ModuleConstants.drivingMotorIdleMode);
    drivingSparkMax.setSmartCurrentLimit(
        (int) Constants.ModuleConstants.drivingMotorCurrentLimit.in(Amps));
  }

  private void configureTurningController() {
    turningSparkMax.restoreFactoryDefaults();
    turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(
        Constants.ModuleConstants.turningEncoderPositionFactor.in(Radians));
    turningEncoder.setVelocityConversionFactor(
        Constants.ModuleConstants.turningEncoderVelocityFactor.in(RadiansPerSecond));

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turningEncoder.setInverted(Constants.ModuleConstants.turningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(
        Constants.ModuleConstants.turningEncoderPositionPIDMinInput.in(Radians));
    turningPIDController.setPositionPIDWrappingMaxInput(
        Constants.ModuleConstants.turningEncoderPositionPIDMaxInput.in(Radians));

    // Set the PID gains for the turning motor.
    turningPIDController.setP(Constants.ModuleConstants.turningP);
    turningPIDController.setI(Constants.ModuleConstants.turningI);
    turningPIDController.setD(Constants.ModuleConstants.turningD);
    turningPIDController.setFF(Constants.ModuleConstants.turningFF);
    turningPIDController.setOutputRange(
        Constants.ModuleConstants.turningMinOutput, Constants.ModuleConstants.turningMaxOutput);

    turningSparkMax.setIdleMode(Constants.ModuleConstants.turningMotorIdleMode);
    turningSparkMax.setSmartCurrentLimit(
        (int) Constants.ModuleConstants.turningMotorCurrentLimit.in(Amps));
  }

  @Override
  public double getWheelVelocity() {
    return drivingEncoder.getVelocity();
  }

  @Override
  public double getWheelDistance() {
    return drivingEncoder.getPosition();
  }

  @Override
  public Rotation2d getModuleRotation() {
    return Rotation2d.fromRadians(turningEncoder.getPosition());
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(
        state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void stop() {
    // Write 0 volts to the motor controllers
    // TODO: Confirm this won't be overridden by the onboard PID controllers
    drivingSparkMax.setVoltage(0);
    turningSparkMax.setVoltage(0);
    drivingPIDController.setReference(0, CANSparkBase.ControlType.kVoltage);
    turningPIDController.setReference(0, CANSparkBase.ControlType.kVoltage);
  }

  @Override
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

  @Override
  public void close() {
    drivingSparkMax.close();
    turningSparkMax.close();
  }

  public double getTurnVoltage() {
    return turningSparkMax.getAppliedOutput();
  }

  @Override
  public void setDrivingMotorVoltage(Measure<Voltage> v) {
    drivingSparkMax.setVoltage(v.in(Volts));
  }
}
