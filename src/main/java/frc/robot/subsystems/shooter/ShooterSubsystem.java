package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax rSpark =
      new CANSparkMax(Constants.ShooterConstants.rightShooterCanId, MotorType.kBrushless);
  private final CANSparkMax lSpark =
      new CANSparkMax(Constants.ShooterConstants.leftShooterCanId, MotorType.kBrushless);

  public ShooterSubsystem() {
    rSpark.restoreFactoryDefaults();
    lSpark.restoreFactoryDefaults();
    lSpark.setInverted(true);
  }

  public void spin() {
    // spark.setVoltage(SmartDashboard.getNumber("Spin voltage", 0));
    rSpark.setVoltage(12);
    lSpark.setVoltage(11);
    System.out.println("spinning motors");
  }

  public void ampShot() {
    rSpark.setVoltage(2.2);
    lSpark.setVoltage(2.2);
  }

  public void stopSpin() {
    rSpark.set(0);
    lSpark.set(0);
    System.out.println("Stopping spin");
  }

  public void hold() {
    rSpark.setVoltage(SmartDashboard.getNumber("Hold voltage", 0));
    lSpark.setVoltage(SmartDashboard.getNumber("Hold voltage", 0));
  }

  public void actuate() {}

  public Command manualShootCommand() {
    System.out.println("shoot commanded");
    return run(this::spin).finallyDo(interrupted -> stopSpin());
  }

  public Command autoShootCommand() {
    System.out.println("shoot commanded");
    // return run(this::spin).finallyDo(interrupted -> stopSpin());
    return run(this::spin).withTimeout(4).finallyDo(interrupted -> stopSpin());
  }

  public Command holdCommand() {
    return run(this::hold);
  }

  public Command stopSpinCommand() {
    return runOnce(this::stopSpin);
  }

  public Command ampCommand() {
    return run(this::ampShot).finallyDo(interrupted -> stopSpin());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left velocity", lSpark.getEncoder().getVelocity());
  }
}
