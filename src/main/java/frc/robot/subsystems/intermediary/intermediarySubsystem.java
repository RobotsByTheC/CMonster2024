package frc.robot.subsystems.intermediary;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntermediarySubsystem extends SubsystemBase {

  private final CANSparkMax spark =
      new CANSparkMax(Constants.IntermediaryConstants.intermediaryCanID, MotorType.kBrushless);

  public void spin() {
    spark.set(Constants.IntermediaryConstants.intermediaryRotationalSpeed);
  }

  public void stopSpin() {
    spark.set(0);
  }

  public void hold() {
    spark.setVoltage(SmartDashboard.getNumber("Hold voltage", 0));
  }

  public void actuate() {}

  public Command intermediaryCommand() {
    return run(this::spin).finallyDo(interrupted -> stopSpin());
  }

  public Command holdCommand() {
    return run(this::hold);
  }

  public Command stopSpinCommand() {
    return runOnce(this::stopSpin);
  }
}
