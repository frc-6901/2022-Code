// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  // Spark Max Motor Controller Object
  private CANSparkMax m_shooterMotorLeader =
      new CANSparkMax(ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);
  private CANSparkMax m_shooterMotorFollower =
      new CANSparkMax(ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);

  // Spark Max PID Controller Object
  private SparkMaxPIDController m_shooterController = m_shooterMotorLeader.getPIDController();

  // Feed Forward Calculator
  private SimpleMotorFeedforward m_flywheelFeedforward =
      new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

  /** Creates a new Shooter */
  public Shooter() {
    m_shooterMotorLeader.setInverted(true);
    m_shooterMotorFollower.follow(m_shooterMotorLeader, true);
    m_shooterController.setP(ShooterConstants.kP);
  }

  // Sets the RPM to the specified parameter
  public void setRPM(int RPM) {
    double feedForward = m_flywheelFeedforward.calculate(RPM / 60);
    m_shooterController.setReference(
        RPM, ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
  }

  public boolean atTargetRPM() {
    double actualRPM = m_shooterMotorLeader.getEncoder().getVelocity();
    return Math.abs(actualRPM - ShooterConstants.kTargetRPM) <= ShooterConstants.kRPMThreshold;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", m_shooterMotorLeader.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
