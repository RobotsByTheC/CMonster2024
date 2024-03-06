package frc.robot.subsystems.intermediary;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intermediarySubsystem extends SubsystemBase {

  private final CANSparkMax spark =
      new CANSparkMax(Constants.IntermediaryConstants.intermediaryCanID, MotorType.kBrushless);

  public void spin() {
    spark.set(Constants.IntermediaryConstants.intermediaryRotationalSpeed);
  }

  public void reverseSpin() {
    spark.set(Constants.IntermediaryConstants.intermediaryBackRotationalSpeed);
  }

  public void spinBack() {
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
    return run(this::spin).withTimeout(3);
  }
  
  
  public Command intermediaryReverseCommand() {
    return run(this::reverseSpin).withTimeout(.5).finallyDo(interrupted -> stopSpin());
  }

  public Command holdCommand() {
    return run(this::hold);
  }

  public Command stopSpinCommand() {
    return runOnce(this::stopSpin);
  }

  public Command handOutCommand() {
    return run(this::spin)
        .withTimeout(3)
        .finallyDo(
            interrupted ->
                stopSpin()); // basically just shove it outwards and take 3 seconds doing it so
    // shooter has time to rev up
  }

  public Command handInCommand() {
    return run(this::spinBack); // .until(lowerBeamBroken) or something like that
  }
}
