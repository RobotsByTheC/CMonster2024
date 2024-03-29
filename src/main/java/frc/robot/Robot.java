// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.cameraserver.CameraServer;
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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.sim.SimulationContext;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.SimSwerveIO;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intermediary.IntermediarySubsystem;
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
  private IntermediarySubsystem intermediary;
  private ClimberSubsystem climber;

  // Driver and operator controls
  private XboxController driverController;
  private Joystick lStick; // NOPMD
  private Joystick rStick; // NOPMD

  enum Positions {
    STAGE,
    CENTER,
    AMP,
    NEXT2STAGE,
    NEXT2AMP
  }

  enum Notes {
    STAGE,
    CENTER,
    AMP,
    NOTHING,
    DRIVE,
    NEXT2STAGE,
    NEXT2AMP,
    GOFORWARDS,
    CENTERSHOOTDRIVE,
    DIAGONALSHOOTDRIVE,
    SABOTAGE
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

    var driverCamera = CameraServer.startAutomaticCapture();
    // driverCamera.setPixelFormat(PixelFormat.kYUYV);
    driverCamera.setResolution(1280, 720);

    if (Robot.isSimulation()) {
      drive = new DriveSubsystem(new SimSwerveIO());
      intake = new IntakeSubsystem();
      intermediary = new IntermediarySubsystem();
      shooter = new ShooterSubsystem(intermediary::noteCheck);
      climber = new ClimberSubsystem();
    } else {
      // Running on real hardware
      drive = new DriveSubsystem(new MAXSwerveIO());
      intake = new IntakeSubsystem();
      intermediary = new IntermediarySubsystem();
      shooter = new ShooterSubsystem(intermediary::noteCheck);
      climber = new ClimberSubsystem();
    }
    leds = new LEDSubsystem();

    driverController = new XboxController(Constants.OIConstants.driverControllerPort);
    lStick = new Joystick(Constants.OIConstants.leftJoystickPort);
    rStick = new Joystick(Constants.OIConstants.rightJoystickPort);

    // Configure the button bindings and automatic bindings
    configureButtonBindings();
    configureAutomaticBindings();

    // Configure default commands
    drive.setDefaultCommand(drive.driveWithJoysticks(rStick::getY, rStick::getX, lStick::getTwist));

    // Set up autonomous chooser

    startingPositionChooser.setDefaultOption("Stage", Positions.STAGE);
    startingPositionChooser.addOption("Center", Positions.CENTER);
    startingPositionChooser.addOption("Amp", Positions.AMP);
    startingPositionChooser.addOption("next 2 amp", Positions.NEXT2AMP);
    startingPositionChooser.addOption("next 2 stage", Positions.NEXT2STAGE);
    SmartDashboard.putData("starting position", startingPositionChooser);

    noteChooser1.setDefaultOption("be useless", Notes.NOTHING);
    noteChooser1.addOption("Center Note", Notes.CENTER);
    noteChooser1.addOption("Stage", Notes.STAGE);
    noteChooser1.addOption("Amp Note", Notes.AMP);
    noteChooser1.addOption("Drive", Notes.DRIVE);
    noteChooser1.addOption("Next 2 Amp", Notes.NEXT2AMP);
    noteChooser1.addOption("Next 2 Stage", Notes.NEXT2STAGE);
    noteChooser1.addOption("Go Forwards", Notes.GOFORWARDS);
    noteChooser1.addOption("centershootdrive", Notes.CENTERSHOOTDRIVE);
    noteChooser1.addOption("sabotage", Notes.SABOTAGE);
    noteChooser1.addOption("diagonal shoot drive", Notes.DIAGONALSHOOTDRIVE);
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
    /*  new JoystickButton(driverController, PS4Controller.Button.kCross.value)
    .and(DriverStation::isTeleop)
    .whileTrue(drive.setXCommand());*/
    new JoystickButton(driverController, PS4Controller.Button.kL1.value)
        .whileTrue(shooter.manualShootCommand())
        .whileTrue(leds.blinkGreen())
        .onFalse(leds.greenPurpleScroll());
    new JoystickButton(driverController, PS4Controller.Button.kTriangle.value)
        .whileTrue(intermediary.intermediaryCommand())
        .whileTrue(intake.intakeCommand())
        .onFalse(intermediary.intermediaryReverseCommand().withTimeout(.2));
    new JoystickButton(driverController, PS4Controller.Button.kSquare.value)
        .whileTrue(shooter.ampCommand().deadlineWith(leds.blinkPurple()))
        .onFalse(shooter.stopSpinCommand());
    new JoystickButton(driverController, PS4Controller.Button.kCross.value)
        .and(DriverStation::isTest)
        .whileTrue(
            drive
                .sysIdDynamic(Direction.kForward)
                .andThen(drive.sysIdQuasistatic(Direction.kForward))
                .andThen(drive.sysIdDynamic(Direction.kReverse))
                .andThen(drive.sysIdQuasistatic(Direction.kReverse)));
    new JoystickButton(driverController, PS4Controller.Button.kCircle.value)
        .whileTrue(intake.spinReverseCommand())
        .whileTrue(shooter.reverseShooterCommand())
        .whileTrue(intermediary.intermediaryReverseCommand());
    new JoystickButton(driverController, PS4Controller.Button.kR1.value)
        .whileTrue(climber.climbCommand())
        .onFalse(climber.stopClimbCommand());
    new JoystickButton(driverController, PS4Controller.Button.kCross.value)
        .whileTrue(climber.reverseClimbCommand())
        .onFalse(climber.stopClimbCommand());
  }

  private void configureAutomaticBindings() {
    new Trigger(intermediary::noteCheck).onTrue(leds.blinkRed().withTimeout(1));
    new Trigger(shooter::atSpeakerSpeed).onTrue(leds.rainbowFlagScroll());
    new Trigger(shooter::atAmpSpeed).onTrue(leds.rainbowFlagScroll());
  }

  /**
   * Gets the command to run in autonomous based on user selection in a dashboard.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return switch (startingPositionChooser.getSelected()) {
      case AMP -> {
        Command auto = speakerShot();
        boolean done = false;
        switch (noteChooser1.getSelected()) {
          case AMP -> auto = auto.andThen(followPathAndShoot("amp 3 p1"));
          case CENTER -> auto = auto.andThen(followPathAndShoot("amp 3 p2"));
          case STAGE -> auto = auto.andThen(followPathAndShoot("amp 3 p3"));
          case DRIVE -> {
            auto = auto.andThen(drive.followChoreoTrajectory("amp drive"));
            done = true;
          }
          case DIAGONALSHOOTDRIVE -> auto = auto.andThen(deadReckoningDiagonal());
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
        Command auto = speakerShot();
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
          case CENTERSHOOTDRIVE -> {
            auto = auto.andThen(deadReckoningForward());
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
        Command auto = speakerShot();
        boolean done = false;
        switch (noteChooser1.getSelected()) {
          case AMP -> auto = auto.andThen(followPathAndShoot("stage 3 p1"));
          case CENTER -> auto = auto.andThen(followPathAndShoot("stage 3 p2"));
          case STAGE -> auto = auto.andThen(followPathAndShoot("stage 3 p3"));
          case DRIVE -> {
            auto = auto.andThen(drive.followChoreoTrajectory("stage drive"));
            done = true;
          }
          case DIAGONALSHOOTDRIVE -> auto = auto.andThen(deadReckoningDiagonal());
          case NOTHING -> {
            done = true;
          }
          case GOFORWARDS -> {
            auto = drive.followChoreoTrajectory("goForwards");
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

      case NEXT2AMP -> {
        Command auto = speakerShot();
        boolean done = false;
        switch (noteChooser1.getSelected()) {
          case NOTHING -> {
            done = true;
          }
          case SABOTAGE -> {
            auto = sabotage();
          }
          default -> {
            done = true;
          }
          case NEXT2AMP -> {
            auto = followPathAndShoot("next 2 amp");
          }
        }
        yield auto;
      }
      case NEXT2STAGE -> {
        Command auto = speakerShot();
        boolean done = false;
        switch (noteChooser1.getSelected()) {
          case NOTHING -> {
            done = true;
          }
          default -> {
            done = true;
          }
          case NEXT2STAGE -> {
            auto = followPathAndShoot("next 2 stage");
          }
        }
        yield auto;
      }
    };
  }

  private SequentialCommandGroup followPathAndShoot(String p) {
    return drive
        .followChoreoTrajectory(p)
        .andThen(drive.setXCommand().withTimeout(1))
        .deadlineWith(intake.intakeCommand())
        .andThen(
            shooter
                .autoShootCommand1()
                .andThen(
                    shooter.autoShootCommand2().deadlineWith(intermediary.intermediaryCommand())));
  }

  private Command speakerShot() {
    return intermediary
        .intermediaryReverseCommand()
        .withTimeout(.5)
        .andThen(
            shooter
                .autoShootCommand1()
                .deadlineWith(drive.setXCommand())
                .andThen(
                    shooter.autoShootCommand2().deadlineWith(intermediary.intermediaryCommand())));
  }

  private SequentialCommandGroup deadReckoningForward() {
    return new PrintCommand("deadReckoningForward")
        .andThen(drive.pointForward().withTimeout(1.5))
        .andThen(new PrintCommand("gonna drive now"))
        .andThen(
            drive
                .autoDriveForwardCommand()
                .alongWith(intermediary.intermediaryCommand())
                .alongWith(intake.intakeCommand()))
        .withTimeout(1.1)
        .andThen(drive.autoDriveBackwardCommand().withTimeout(1.1))
        .andThen(speakerShot());
  }

  private Command sabotage() {
    return drive
        .pointForward()
        .withTimeout(1.5)
        .andThen(new PrintCommand("gonna drive now"))
        .andThen(drive.autoDriveForwardCommand())
        .withTimeout(.143)
        .andThen(drive.autoTurn120())
        .andThen(drive.autoDriveForwardCommand())
        .withTimeout(3.5)
        .andThen(drive.autoTurn90())
        .andThen(drive.autoDriveForwardCommand())
        .alongWith(intermediary.intermediaryCommand())
        .alongWith(intake.intakeCommand())
        .alongWith(shooter.ampCommand())
        .withTimeout(3.5);
  }

  private Command threeNote() {
    return deadReckoningForward()
        .andThen(drive.autoDriveForwardCommand().withTimeout(1.1))
        .andThen(drive.autoTurn90())
        .andThen(drive.autoDriveForwardCommand())
        .alongWith(intake.intakeCommand())
        .alongWith(intermediary.intermediaryCommand())
        .withTimeout(0.7)
        .andThen(drive.autoDriveBackwardCommand())
        .withTimeout(.7)
        .andThen(drive.autoTurnNeg90())
        .andThen(drive.autoDriveBackwardCommand())
        .withTimeout(1.1)
        .andThen(speakerShot())
        .andThen(drive.autoDriveForwardCommand().withTimeout(.143))
        .andThen(drive.autoDriveSidewaysCommand().withTimeout(.7))
        .andThen(
            drive
                .autoDriveForwardCommand()
                .alongWith(intake.intakeCommand().alongWith(intermediary.intermediaryCommand()))
                .withTimeout(1.4))
        .andThen(drive.autoDriveBackwardCommand().withTimeout(1.4))
        .andThen(drive.autoDriveSidewaysCommand().withTimeout(.7))
        .andThen(drive.autoDriveForwardCommand().withTimeout(.143))
        .andThen(speakerShot());
  }

  private SequentialCommandGroup deadReckoningDiagonal() {
    return drive
        .pointForward()
        .withTimeout(1.5)
        .andThen(new PrintCommand("gonna drive now"))
        .andThen(drive.autoDriveDiagonalCommand())
        .andThen(drive.setXCommand());
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
  public void disabledInit() {
    // leds.blinkYellow().schedule();
    leds.greenPurpleGradient().schedule();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by the {@link #autoChooser}. */
  @Override
  public void autonomousInit() {
    leds.police().schedule();
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

    leds.greenPurpleScroll().schedule();
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
