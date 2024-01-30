// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.SimSwerveIO;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  // The robot's subsystems
  private DriveSubsystem robotDrive;
  private ShooterSubsystem robotShoot;
  private IntakeSubsystem robotIntake; // NOPMD

  // Driver and operator controls
  private XboxController driverController;
  private Joystick lStick; // NOPMD
  private Joystick rStick; // NOPMD

  enum AutoType {
    NOTHING,
    SPRINT,
    AMP,
    CENTER,
    STAGE,
    GREED,
  }

  /** Used to select a preplanned autonomous routine on a dashboard. */
  private final SendableChooser<AutoType> autoChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Initialize our subsystems. If our program is running in simulation mode (either from the
    // simulate command in vscode or from running in unit tests), then we use the simulation IO
    // layers. Otherwise, the IO layers that interact with real hardware are used.
    if (Robot.isSimulation()) {
      robotDrive = new DriveSubsystem(new SimSwerveIO());
      robotShoot = new ShooterSubsystem();
      robotIntake = new IntakeSubsystem();
    } else {
      // Running on real hardware
      robotDrive = new DriveSubsystem(new MAXSwerveIO());
      robotShoot = new ShooterSubsystem();
      robotIntake = new IntakeSubsystem();
    }

    driverController = new XboxController(Constants.OIConstants.driverControllerPort);
    lStick = new Joystick(Constants.OIConstants.leftJoystickPort);
    rStick = new Joystick(Constants.OIConstants.rightJoystickPort);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    robotDrive.setDefaultCommand(
        robotDrive.driveWithJoysticks(rStick::getY, rStick::getX, lStick::getTwist));

    // Set up autonomous chooser
    autoChooser.setDefaultOption("Sit Still And Be Useless", AutoType.NOTHING);
    autoChooser.addOption("Sprint", AutoType.SPRINT);
    autoChooser.addOption("Greedy Notes", AutoType.GREED);
    autoChooser.addOption("Amp", AutoType.AMP);
    autoChooser.addOption("Stage", AutoType.STAGE);
    autoChooser.addOption("Center", AutoType.CENTER);
    SmartDashboard.putData("Autonomous Mode", autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, PS4Controller.Button.kR1.value)
        .whileTrue(robotDrive.setXCommand());
    new JoystickButton(driverController, PS4Controller.Button.kTriangle.value)
        .whileTrue(robotShoot.shootCommand());
  }

  /**
   * Gets the command to run in autonomous based on user selection in a dashboard.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return switch (autoChooser.getSelected()) {
      case SPRINT -> robotDrive.followChoreoTrajectory("sprint");
      case GREED -> robotDrive.followChoreoTrajectory("greedy_notes");
      case AMP -> robotDrive.followChoreoTrajectory("amp pickup");
      case CENTER -> robotDrive.followChoreoTrajectory("center pickup");
      case STAGE -> robotDrive.followChoreoTrajectory("stage pickup");
      case NOTHING -> robotDrive.run(() -> {});
    };
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Send electrical information to the dashboard
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Current Draw", RobotController.getInputCurrent());
  }

  @Override
  public void simulationPeriodic() {
    SimulationContext.getDefault().update(getPeriod());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by the {@link #autoChooser}. */
  @Override
  public void autonomousInit() {
    autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
