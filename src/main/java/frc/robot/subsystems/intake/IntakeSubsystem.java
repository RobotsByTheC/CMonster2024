
package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.LauncherConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.Angle;

public class IntakeSubsystem extends SubsystemBase
  {

    private final CANSparkMax spark = new CANSparkMax(Constants.IntakeConstants.intakeCanId, MotorType.kBrushless);
    
    public void spin() {
        spark.setVoltage(SmartDashboard.getNumber("Spin voltage", 0));
        //spark.setRPM(Constants.IntakeConstants.intakeRotationalSpeed);
    }

    public void stopSpin() {
        spark.set(0);
    }

    public void hold() {
        spark.setVoltage(SmartDashboard.getNumber("Hold voltage", 0));
    }

    public void actuate() {

    }

    public Command spinCommand() {
        return run(this::spin).finallyDo((interrupted)->stopSpin());

    }

    public Command holdCommand() {
        return run(this::hold);
    }

    public Command stopSpinCommand() {
        return runOnce(this::stopSpin);
    }

  }

