// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.OneBallAuto;
import frc.robot.commands.ResetShooter;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TwoBallAuto;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticClimb;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_subsystem =
      new ExampleSubsystem(); // you said not to mess with this
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final PneumaticClimb m_pneumaticClimb = new PneumaticClimb();

  private final Indexer m_indexer = new Indexer();

  private final Drivetrain m_drivetrain = new Drivetrain(m_indexer.getPigeon());

  private final Climb m_climb = new Climb();

  private final OneBallAuto m_oneBallAuto = new OneBallAuto(m_drivetrain, m_shooter, m_indexer);

  private final TwoBallAuto m_twoBallAuto =
      new TwoBallAuto(m_drivetrain, m_shooter, m_indexer, m_intake);

  private final Command m_crossWithTimer =
      new RunCommand(
              () -> {
                m_drivetrain.drive(0.3, 0.0);
              },
              m_drivetrain)
          .withTimeout(DrivetrainConstants.kAutoTime)
          .andThen(
              () -> {
                m_drivetrain.drive(0.0, 0.0);
              },
              m_drivetrain);

  private final Command m_noAuto =
      new InstantCommand(
          () -> {
            m_drivetrain.drive(0.0, 0.0);
          },
          m_drivetrain);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private final XboxController m_navigatorController =
      new XboxController(ControllerConstants.kNavigatorPort);
  private final XboxController m_operatorController =
      new XboxController(ControllerConstants.kOperatorPort);
  private Trajectory trajectory = new Trajectory();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(
        new RunCommand(
            () -> {
              m_drivetrain.drive(
                  -DrivetrainConstants.kDriveForwardMultiplier * m_navigatorController.getLeftY(),
                  DrivetrainConstants.kDriveTurnMultiplier * m_navigatorController.getRightX());
            },
            m_drivetrain));

    m_indexer.setDefaultCommand(
        new RunCommand(
            () -> {
              m_indexer.setState(IndexerState.kPassive);
            },
            m_indexer));

    m_climb.setDefaultCommand(
        (new RunCommand(
            () -> {
              m_climb.setClimb(0.0);
            },
            m_climb)));
    // Configure the button bindings
    configureButtonBindings();

    m_autoChooser.setDefaultOption("No Auto", m_noAuto);
    m_autoChooser.addOption("Cross the line", m_crossWithTimer);
    m_autoChooser.addOption("Single Ball", m_oneBallAuto);
    m_autoChooser.addOption("Two Ball", m_twoBallAuto);
    Shuffleboard.getTab("Autonomous").add(m_autoChooser);
  }

  public void setTrajectory(Trajectory t) {
    trajectory = t;
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_operatorController, Button.kX.value)
        .whileHeld(new ShootCommand(ShooterConstants.kShooterFenderRPM, m_shooter, m_indexer))
        .whenReleased(new ResetShooter(m_shooter, m_indexer));

    new JoystickButton(m_operatorController, Button.kLeftBumper.value)
        .whenPressed(
            () -> {
              m_intake.extendIntake();
            },
            m_intake);

    new JoystickButton(m_operatorController, Button.kRightBumper.value)
        .whenPressed(
            () -> {
              m_intake.retractIntake();
            },
            m_intake);

    new JoystickButton(m_navigatorController, Button.kLeftBumper.value)
        .whenPressed(
            () -> {
              m_intake.intakeBalls();
            },
            m_intake)
        .whenReleased(
            () -> {
              m_intake.noBalls();
            });

    new JoystickButton(m_navigatorController, Button.kRightBumper.value)
        .whenPressed(
            () -> {
              m_intake.outtakeBalls();
            },
            m_intake)
        .whenReleased(
            () -> {
              m_intake.noBalls();
            });

    new JoystickButton(m_operatorController, Button.kB.value)
        .whileHeld(
            () -> {
              m_indexer.setState(IndexerState.kFeeding);
            },
            m_indexer)
        .whenReleased(
            () -> {
              m_indexer.setState(IndexerState.kPassive);
            },
            m_indexer);

    new JoystickButton(m_operatorController, Button.kA.value)
        .whileHeld(
            () -> {
              m_indexer.setState(IndexerState.kReverse);
            },
            m_indexer)
        .whenReleased(
            () -> {
              m_indexer.setState(IndexerState.kPassive);
            },
            m_indexer);
    new POVButton(m_operatorController, 90)
        .whenPressed(
            () -> {
              m_pneumaticClimb.extendClimb();
            },
            m_pneumaticClimb);

    new POVButton(m_operatorController, 270)
        .whenPressed(
            () -> {
              m_pneumaticClimb.retractClimb();
            },
            m_pneumaticClimb);
    new POVButton(m_operatorController, 0)
        .whileHeld(
            () -> {
              m_climb.setClimb(ClimbConstants.kClimbPower);
            },
            m_climb);
    new POVButton(m_operatorController, 180)
        .whileHeld(
            () -> {
              m_climb.setClimb(-ClimbConstants.kClimbPower);
            },
            m_climb);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
