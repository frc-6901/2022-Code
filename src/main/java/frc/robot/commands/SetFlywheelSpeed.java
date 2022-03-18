// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Shooter;

public class SetFlywheelSpeed extends CommandBase {
  Shooter m_shooter;
  Indexer m_indexer;
  double m_targetRPM = 0.0;
  /** Creates a new ShootCommand. */
  public SetFlywheelSpeed(double RPM, Shooter shooter, Indexer indexer) {
    m_targetRPM = RPM;
    m_shooter = shooter;
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.setState(IndexerState.kStopped);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setRPM(m_targetRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_shooter.getRPM() - m_targetRPM) <= ShooterConstants.kShooterRPMThreshold;
  }
}
