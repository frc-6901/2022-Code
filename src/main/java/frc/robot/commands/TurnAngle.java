// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnAngle extends ProfiledPIDCommand {

  /** Creates a new TurnAngle. */
  public TurnAngle(double angle, Drivetrain drive) {

    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            DrivetrainConstants.kAngularKP,
            0,
            DrivetrainConstants.kAngularKD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                DrivetrainConstants.kMaxTurnRate, DrivetrainConstants.kMaxTurnAccel)),
        // This should return the measurement
        drive::getHeading,
        // This should return the goal (can also be a constant)
        angle,
        // This uses the output
        (output, setpoint) -> {
          drive.drive(0.0, -output);
        },
        drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(DrivetrainConstants.kDegreeTolerance);
    addRequirements(drive);
    drive.resetGyro();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
