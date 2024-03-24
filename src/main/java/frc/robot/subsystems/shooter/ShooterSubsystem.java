package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.NeoMotorConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax rSpark =
      new CANSparkMax(Constants.ShooterConstants.rightShooterCanId, MotorType.kBrushless);
  private final CANSparkMax lSpark =
      new CANSparkMax(Constants.ShooterConstants.leftShooterCanId, MotorType.kBrushless);
private final SparkPIDController rSparkPID = rSpark.getPIDController();
  private final SparkPIDController lSparkPID = lSpark.getPIDController();
  private final RelativeEncoder rSparkEncoder = rSpark.getEncoder();
  private final RelativeEncoder lSparkEncoder = lSpark.getEncoder();
  public final Trigger atSpeakerSpeed = new Trigger(this::atSpeakerSpeed);
  private final BooleanSupplier hasNote;

  public ShooterSubsystem(BooleanSupplier hasNote) {
this.hasNote = hasNote;

    rSpark.restoreFactoryDefaults();
    lSpark.restoreFactoryDefaults();
    lSpark.setInverted(true);
rSparkPID.setP(.00);
    rSparkPID.setI(0);
    rSparkPID.setD(0);
    rSparkPID.setFF((1 / NeoMotorConstants.freeSpeedRpm.in(RPM))+.00002);
    lSparkPID.setP(.00);
    lSparkPID.setI(0);
    lSparkPID.setD(0);
    lSparkPID.setFF((1 / NeoMotorConstants.freeSpeedRpm.in(RPM))+.00002);

    Shuffleboard.getTab("Shooter").addBoolean("at speaker speed", atSpeakerSpeed);
    Shuffleboard.getTab("Shooter").addNumber("L shooter speed", lSparkEncoder::getVelocity);
    Shuffleboard.getTab("Shooter").addNumber("R shooter speed", rSparkEncoder::getVelocity);
    Shuffleboard.getTab("Shooter").addBoolean("at amp speed", this::atAmpSpeed);

  }

  public void spin() {
    // spark.setVoltage(SmartDashboard.getNumber("Spin voltage", 0));
    rSparkPID.setReference(
        NeoMotorConstants.freeSpeedRpm.in(RPM), CANSparkMax.ControlType.kVelocity);
    lSparkPID.setReference(
        NeoMotorConstants.freeSpeedRpm.in(RPM) * 0.95, CANSparkMax.ControlType.kVelocity);
    System.out.println("spinning motors");
  }

  public void reverseSpin() {
    rSparkPID.setReference(
        -1700, CANSparkMax.ControlType.kVelocity);
    lSparkPID.setReference(
        -1700, CANSparkMax.ControlType.kVelocity);
  }

  public boolean atSpeakerSpeed() {
    boolean rightAtSpeed;
    boolean leftAtSpeed;
    if (rSparkEncoder.getVelocity() < NeoMotorConstants.freeSpeedRpm.in(RPM) * .905)
      rightAtSpeed = false;
    else rightAtSpeed = true;
    if (lSparkEncoder.getVelocity() < NeoMotorConstants.freeSpeedRpm.in(RPM) * .95 * 0.905)
      leftAtSpeed = false;
    else 
    leftAtSpeed = true;
    return rightAtSpeed && leftAtSpeed;
  }

  public boolean atAmpSpeed() {
    boolean rightAtSpeed;
    boolean leftAtSpeed;
    if (rSparkEncoder.getVelocity() > Constants.ShooterConstants.ampSpeed * .9 && rSparkEncoder.getVelocity() < Constants.ShooterConstants.ampSpeed*1.1)
      rightAtSpeed = true;
    else rightAtSpeed = false;
    if (lSparkEncoder.getVelocity() > Constants.ShooterConstants.ampSpeed * .9 && lSparkEncoder.getVelocity() < Constants.ShooterConstants.ampSpeed*1.1) // 1040.6 is amp speed
      leftAtSpeed = true;
    else leftAtSpeed = false;
    return rightAtSpeed && leftAtSpeed;
  }

  public void ampShot() {
    rSparkPID.setReference(
        Constants.ShooterConstants.ampSpeed, CANSparkMax.ControlType.kVelocity);
    lSparkPID.setReference(
        Constants.ShooterConstants.ampSpeed, CANSparkMax.ControlType.kVelocity);
    System.out.println("spinning motors");
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

  public Command autoShootCommand1() {
    System.out.println("shoot commanded");
    // return run(this::spin).finallyDo(interrupted -> stopSpin());
    return run(this::spin).until(this::atSpeakerSpeed).withTimeout(2);
  }

  public Command reverseShooterCommand() {
    return run(this::reverseSpin).finallyDo(interrupted -> stopSpin());
  }

  public Command autoShootCommand2() {
    System.out.println("shoot commanded");
    // return run(this::spin).finallyDo(interrupted -> stopSpin());
    return run(this::spin).until(() -> !hasNote.getAsBoolean()).withTimeout(2).finallyDo(interrupted -> stopSpin());
  }

  public Command autoAlwaysShootCommand() {
    System.out.println("shoot commanded");
    // return run(this::spin).finallyDo(interrupted -> stopSpin());
    return run(this::spin);
  }

  public Command holdCommand() {
    return run(this::hold);
  }

  public Command stopSpinCommand() {
    return runOnce(this::stopSpin);
  }

  public Command ampCommand() {
    return run(this::ampShot);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left velocity", lSpark.getEncoder().getVelocity());
  }
}
