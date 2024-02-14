// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.SimSwerveIO;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
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
  private LEDSubsystem leds;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private DriveSubsystem drive;

  // Driver and operator controls
  private XboxController driverController;
  private Joystick lStick; // NOPMD
  private Joystick rStick; // NOPMD

  enum Positions {
    STAGE,
    CENTER,
    AMP
  }

  enum Notes {
    STAGE,
    CENTER,
    AMP,
    NOTHING,
    DRIVE
  }

  private final SendableChooser<Positions> startingPositionChooser = new SendableChooser<>();
  private final SendableChooser<Notes> noteChooser1 = new SendableChooser<>();
  private final SendableChooser<Notes> noteChooser2 = new SendableChooser<>();
  private final SendableChooser<Notes> noteChooser3 = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Initialize our subsystems. If our program is running in simulation mode (either from the
    // simulate command in vscode or from running in unit tests), then we use the simulation IO
    // layers. Otherwise, the IO layers that interact with real hardware are used.

    AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

    if (Robot.isSimulation()) {
      drive = new DriveSubsystem(new SimSwerveIO());
      shooter = new ShooterSubsystem();
      intake = new IntakeSubsystem();
    } else {
      // Running on real hardware
      drive = new DriveSubsystem(new MAXSwerveIO());
      shooter = new ShooterSubsystem();
      intake = new IntakeSubsystem();
    }
    leds = new LEDSubsystem();

    driverController = new XboxController(Constants.OIConstants.driverControllerPort);
    lStick = new Joystick(Constants.OIConstants.leftJoystickPort);
    rStick = new Joystick(Constants.OIConstants.rightJoystickPort);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    drive.setDefaultCommand(drive.driveWithJoysticks(rStick::getY, rStick::getX, lStick::getTwist));

    // Set up autonomous chooser

    startingPositionChooser.setDefaultOption("Stage", Positions.STAGE);
    startingPositionChooser.addOption("Center", Positions.CENTER);
    startingPositionChooser.addOption("Amp", Positions.AMP);
    SmartDashboard.putData("starting position", startingPositionChooser);

    noteChooser1.setDefaultOption("be useless", Notes.NOTHING);
    noteChooser1.addOption("Center Note", Notes.CENTER);
    noteChooser1.addOption("Stage", Notes.STAGE);
    noteChooser1.addOption("Amp Note", Notes.AMP);
    noteChooser1.addOption("Drive", Notes.DRIVE);
    SmartDashboard.putData("note chooser 1", noteChooser1);

    noteChooser2.setDefaultOption("be useless", Notes.NOTHING);
    noteChooser2.addOption("Center Note", Notes.CENTER);
    noteChooser2.addOption("Stage", Notes.STAGE);
    noteChooser2.addOption("Amp Note", Notes.AMP);
    noteChooser2.addOption("Drive", Notes.DRIVE);
    SmartDashboard.putData("note chooser 2", noteChooser2);

    noteChooser3.setDefaultOption("be useless", Notes.NOTHING);
    noteChooser3.addOption("Center Note", Notes.CENTER);
    noteChooser3.addOption("Stage", Notes.STAGE);
    noteChooser3.addOption("Amp Note", Notes.AMP);
    noteChooser3.addOption("Drive", Notes.DRIVE);
    SmartDashboard.putData("note chooser 3", noteChooser3);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, PS4Controller.Button.kR1.value)
        .whileTrue(drive.setXCommand());
    new JoystickButton(driverController, PS4Controller.Button.kTriangle.value)
        .whileTrue(shooter.shootCommand().deadlineWith(leds.rainbowFlagScroll()));
    new JoystickButton(driverController, PS4Controller.Button.kSquare.value)
        .whileTrue(
            intake
                .intakeCommand()
                .alongWith(shooter.shootCommand())
                .deadlineWith(leds.blinkPurple()));
    new JoystickButton(driverController, PS4Controller.Button.kCross.value)
        .and(DriverStation::isTest)
        .whileTrue(
            drive
                .sysIdDynamic(Direction.kForward)
                .andThen(drive.sysIdDynamic(Direction.kReverse))
                .andThen(drive.sysIdQuasistatic(Direction.kForward))
                .andThen(drive.sysIdQuasistatic(Direction.kReverse)));
  }

  /**
   * Gets the command to run in autonomous based on user selection in a dashboard.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return switch (startingPositionChooser.getSelected()) {
      case AMP -> {
        Command auto = shooter.shootCommand();
        boolean done = false;
        switch (noteChooser1.getSelected()) {
          case AMP -> auto = auto.andThen(followPathAndShoot("amp 3 p1"));
          case CENTER -> auto = auto.andThen(followPathAndShoot("amp 3 p2"));
          case STAGE -> auto = auto.andThen(followPathAndShoot("amp 3 p3"));
          case DRIVE -> {
            auto = auto.andThen(drive.followChoreoTrajectory("amp drive"));
            done = true;
          }
          case NOTHING -> {
            done = true;
          }
          default -> {
            done = true;
          }
        }
        if (!done) {
          switch (noteChooser2.getSelected()) {
            case AMP -> auto = auto.andThen(followPathAndShoot("amp 3 p1"));
            case CENTER -> auto = auto.andThen(followPathAndShoot("amp 3 p2"));
            case STAGE -> auto = auto.andThen(followPathAndShoot("amp 3 p3"));
            case DRIVE -> {
              auto = auto.andThen(drive.followChoreoTrajectory("amp drive"));
              done = true;
            }
            case NOTHING -> {
              done = true;
            }
            default -> {
              done = true;
            }
          }
        }
        if (!done) {
          switch (noteChooser3.getSelected()) {
            case AMP -> auto = auto.andThen(followPathAndShoot("amp 3 p1"));
            case CENTER -> auto = auto.andThen(followPathAndShoot("amp 3 p2"));
            case STAGE -> auto = auto.andThen(followPathAndShoot("amp 3 p3"));
            case DRIVE -> {
              auto = auto.andThen(drive.followChoreoTrajectory("amp drive"));
            }
            case NOTHING -> {}
            default -> {}
          }
        }
        yield auto;
      }
      case CENTER -> {
        Command auto = shooter.shootCommand();
        boolean done = false;
        switch (noteChooser1.getSelected()) {
          case CENTER -> auto = auto.andThen(followPathAndShoot("center 3 p1"));
          case STAGE -> auto = auto.andThen(followPathAndShoot("center 3 p2"));
          case AMP -> auto = auto.andThen(followPathAndShoot("center 3 p3"));
          case DRIVE -> {
            done = true;
          }
          case NOTHING -> {
            done = true;
          }
          default -> {
            done = true;
          }
        }
        if (!done) {
          switch (noteChooser2.getSelected()) {
            case CENTER -> auto = auto.andThen(followPathAndShoot("center 3 p1"));
            case STAGE -> auto = auto.andThen(followPathAndShoot("center 3 p2"));
            case AMP -> auto = auto.andThen(followPathAndShoot("center 3 p3"));
            case DRIVE -> {
              done = true;
            }
            case NOTHING -> {
              done = true;
            }
            default -> {
              done = true;
            }
          }
        }
        if (!done) {

          switch (noteChooser3.getSelected()) {
            case CENTER -> auto = auto.andThen(followPathAndShoot("center 3 p1"));
            case STAGE -> auto = auto.andThen(followPathAndShoot("center 3 p2"));
            case AMP -> auto = auto.andThen(followPathAndShoot("center 3 p3"));
            case DRIVE -> {}
            case NOTHING -> {}
            default -> {}
          }
        }
        yield auto;
      }
      case STAGE -> {
        Command auto = shooter.shootCommand();
        boolean done = false;
        switch (noteChooser1.getSelected()) {
          case AMP -> auto = auto.andThen(followPathAndShoot("stage 3 p1"));
          case CENTER -> auto = auto.andThen(followPathAndShoot("stage 3 p2"));
          case STAGE -> auto = auto.andThen(followPathAndShoot("stage 3 p3"));
          case DRIVE -> {
            auto = auto.andThen(drive.followChoreoTrajectory("stage drive"));
            done = true;
          }
          case NOTHING -> {
            done = true;
          }
          default -> {
            done = true;
          }
        }
        if (!done) {

          switch (noteChooser2.getSelected()) {
            case AMP -> auto = auto.andThen(followPathAndShoot("stage 3 p1"));
            case CENTER -> auto = auto.andThen(followPathAndShoot("stage 3 p2"));
            case STAGE -> auto = auto.andThen(followPathAndShoot("stage 3 p3"));
            case DRIVE -> {
              auto = auto.andThen(drive.followChoreoTrajectory("stage drive"));
              done = true;
            }
            case NOTHING -> {
              done = true;
            }
            default -> {
              done = true;
            }
          }
        }
        if (!done) {

          switch (noteChooser3.getSelected()) {
            case AMP -> auto = auto.andThen(followPathAndShoot("stage 3 p1"));
            case CENTER -> auto = auto.andThen(followPathAndShoot("stage 3 p2"));
            case STAGE -> auto = auto.andThen(followPathAndShoot("stage 3 p3"));
            case DRIVE -> {
              auto = auto.andThen(drive.followChoreoTrajectory("stage drive"));
            }
            case NOTHING -> {}
            default -> {}
          }
        }
        yield auto;
      }
    };
  }

  private SequentialCommandGroup followPathAndShoot(String p) {
    return drive
        .followChoreoTrajectory(p)
        .andThen(drive.setXCommand())
        .deadlineWith(intake.intakeCommand())
        .andThen(shooter.shootCommand());
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
