// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.ArrayList;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public TwoBallAuto(Drivetrain drive, Shooter shooter, Indexer indexer, Intake intake) {
    var forwardTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(Units.feetToMeters(7.75), 0, new Rotation2d(0)),
            drive.getTrajectoryConfig());

    var reverseConfig =
        new TrajectoryConfig(DrivetrainConstants.kAutoMaxSpeed, DrivetrainConstants.kAutoMaxAccel);
    reverseConfig.setReversed(true);
    var backwardTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.feetToMeters(7.75), 0, new Rotation2d(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(0, 0, new Rotation2d(0)),
            reverseConfig);

    var fullTrajectory = forwardTrajectory.concatenate(backwardTrajectory);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new RunCommand(
                () -> {
                  intake.intakeBalls();
                },
                intake),
            drive.getTrajectoryFollowerCommand(fullTrajectory)),
        new ShootCommand(ShooterConstants.kShooterFenderRPM, 4.0, shooter, indexer));
  }
}
