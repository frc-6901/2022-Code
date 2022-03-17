// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommand extends SequentialCommandGroup {
  /** Creates a new ShootCommand. */
  public ShootCommand(double RPM, Shooter shooter, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetFlywheelSpeed(RPM, shooter, indexer),
        new RunCommand(
            () -> {
              indexer.setState(IndexerState.kFeeding);
            },
            indexer));
  }

  public ShootCommand(double RPM, double durationSeconds, Shooter shooter, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetFlywheelSpeed(RPM, shooter, indexer),
        new RunCommand(
                () -> {
                  indexer.setState(IndexerState.kFeeding);
                },
                indexer)
            .withTimeout(durationSeconds));
  }
}
