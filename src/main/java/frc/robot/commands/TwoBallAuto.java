// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public TwoBallAuto(Drivetrain drive, Shooter shooter, Indexer indexer, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShootCommand(ShooterConstants.kShooterFenderRPM, 4.0, shooter, indexer),
        new ParallelCommandGroup(
            new RunCommand(
                () -> {
                  intake.intakeBalls();
                },
                intake),
            new RunCommand(
                    () -> {
                      drive.drive(0.5, 0.0);
                    },
                    drive)
                .withTimeout(DrivetrainConstants.kAutoTime)),
        new InstantCommand(
            () -> {
              intake.retractIntake();
            },
            intake),
        new RunCommand(
                () -> {
                  drive.drive(-0.5, 0.0);
                },
                drive)
            .withTimeout(DrivetrainConstants.kAutoTime),
        new ShootCommand(ShooterConstants.kShooterFenderRPM, 4.0, shooter, indexer));
  }
}
