// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.LimelightManager;
import frc.robot.subsystems.Shooter;
import frc.robot.util.InterpolatingDouble;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightShoot extends SequentialCommandGroup {
  /** Creates a new LimelightShoot. */
  public LimelightShoot(Shooter shooter, Indexer indexer, Hood hood, LimelightManager limelight) {
    double RPM =
        LimelightConstants.kShooterRPMMap.getInterpolated(
                new InterpolatingDouble(limelight.getDistance()))
            .value;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new RunCommand(
                    () -> {
                      hood.setTargetPosition(
                          LimelightConstants.kHoodMap.getInterpolated(
                                  new InterpolatingDouble(limelight.getDistance()))
                              .value);
                    },
                    hood)
                .until(hood::atTargetPosition),
            new SetFlywheelSpeed(RPM, shooter, indexer)
                .withTimeout(ShooterConstants.kRampUpTimeoutSeconds)),
        new RunCommand(
            () -> {
              shooter.setRPM(RPM);
              indexer.setState(IndexerState.kFeeding);
            },
            indexer));
  }
}
