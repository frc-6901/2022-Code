// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
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

  private final ExampleCommand m_autoCommand =
      new ExampleCommand(m_subsystem); // you said not to mess with this

  private final XboxController m_navigatorController =
      new XboxController(ControllerConstants.kNavigatorPort);
  private final XboxController m_operatorController =
      new XboxController(ControllerConstants.kOperatorPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_navigatorController, Button.kX.value)
        .whenPressed(
            () -> {
              m_shooter.setRPM(2800);
            },
            m_shooter)
        .whenReleased(
            () -> {
              m_shooter.setRPM(0);
            },
            m_shooter);

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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
