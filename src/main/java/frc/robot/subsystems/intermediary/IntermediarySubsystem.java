package frc.robot.subsystems.intermediary;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntermediarySubsystem extends SubsystemBase {

  private final DigitalInput beamBreak = new DigitalInput(1);

  private final CANSparkMax spark =
      new CANSparkMax(Constants.IntermediaryConstants.intermediaryCanID, MotorType.kBrushless);

  public boolean noteCheck() {
    return !beamBreak.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beamBreak", this.noteCheck());
  }

  public void spin() {
    spark.setVoltage(Constants.IntermediaryConstants.intermediaryRotationalSpeed);
  }

  public void reverseSpin() {
    spark.set(Constants.IntermediaryConstants.intermediaryBackRotationalSpeed);
  }

  public void stopSpin() {
    spark.set(0);
  }

  public void hold() {
    spark.setVoltage(SmartDashboard.getNumber("Hold voltage", 0));
  }

  public Command intermediaryCommand() {
    return run(this::spin).finallyDo(interrupted -> stopSpin());
  }

  public Command autoIntermediaryCommand() {
    return run(this::spin).until(this::noteCheck);
  }

  public Command intermediaryReverseCommand() {
    return run(this::reverseSpin).finallyDo(interrupted -> stopSpin());
  }

  public Command holdCommand() {
    return run(this::hold);
  }

  public Command stopSpinCommand() {
    return runOnce(this::stopSpin);
  }
}
