// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.SimSwerveIO;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive;

  enum AutoType {
    NOTHING,
    SPRINT,
    GREED
  }

  private final SendableChooser<AutoType> autoChooser = new SendableChooser<>();

  // The driver's controller
  private final XboxController driverController =
      new XboxController(OIConstants.driverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Robot.isSimulation()) {
      robotDrive = new DriveSubsystem(new SimSwerveIO());
    } else {
      robotDrive = new DriveSubsystem(new MAXSwerveIO());
    }

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    robotDrive.setDefaultCommand(
        robotDrive.driveWithJoysticks(
            driverController::getLeftY, driverController::getLeftX, driverController::getRightX));

    SmartDashboard.putNumber("Linear P", 5);
    SmartDashboard.putNumber("Angular P", 2);

    // Set up autonomous chooser
    autoChooser.setDefaultOption("Sit Still And Be Useless", AutoType.NOTHING);
    autoChooser.addOption("Sprint", AutoType.SPRINT);
    autoChooser.addOption("Greedy Notes", AutoType.GREED);

    SmartDashboard.putData("Autonomous Mode", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, Button.kR1.value).whileTrue(robotDrive.setXCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return switch (autoChooser.getSelected()) {
      case NOTHING -> robotDrive.run(() -> {});
      case SPRINT -> robotDrive.followChoreoTrajectory("sprint");
      case GREED -> robotDrive.followChoreoTrajectory("greedy_notes");
    };
  }
}
