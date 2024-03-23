package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax leftSpark =
      new CANSparkMax(Constants.ClimberConstants.climberLeftSpark, MotorType.kBrushless);

  private final CANSparkMax rightSpark =
      new CANSparkMax(Constants.ClimberConstants.climberRightSpark, MotorType.kBrushless);

      

  public void startClimb() {
    // spark.setVoltage(SmartDashboard.getNumber("Spin voltage", 0));
    if (leftSpark.getEncoder().getPosition() < -81)
    {
      stopClimb();
    }
    else 
    {
    rightSpark.set(Constants.ClimberConstants.climberSpeed);
    leftSpark.set(Constants.ClimberConstants.reverseClimberSpeed);
    System.out.println(leftSpark.getEncoder().getPosition());
    }
  }

  public void reverseClimb() {
    if (leftSpark.getEncoder().getPosition() > -0.1)
    {
      stopClimb();
    }
    else 
    {
    rightSpark.set(Constants.ClimberConstants.reverseClimberSpeed);
    leftSpark.set(Constants.ClimberConstants.climberSpeed);
    System.out.println(leftSpark.getEncoder().getPosition());
    }
  }

  public void stopClimb() {
    leftSpark.set(0);
    rightSpark.set(0);
    //Shuffleboard.getTab("left climb distance").add(leftSpark.getEncoder().getPosition());
    System.out.println(leftSpark.getEncoder().getPosition());
    //SmartDashboard.putNumber("right climb distance", rightSpark.getEncoder().getPosition());
  }

  public Command climbCommand() {
    return run(this::startClimb).finallyDo(interrupted -> stopClimb());
  }

  public Command reverseClimbCommand() {
    return run(this::reverseClimb).finallyDo(interrupted -> stopClimb());
  }

  public Command stopClimbCommand() {
    return runOnce(this::stopClimb);
  }
}
