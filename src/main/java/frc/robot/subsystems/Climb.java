// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private CANSparkMax m_leftClimbSide =
      new CANSparkMax(ClimbConstants.kLeftClimberPort, MotorType.kBrushless);
  private CANSparkMax m_rightClimbSide =
      new CANSparkMax(ClimbConstants.kRightClimberPort, MotorType.kBrushless);

  /** Creates a new Climb. */
  public Climb() {
    m_leftClimbSide.restoreFactoryDefaults();
    m_rightClimbSide.restoreFactoryDefaults();

    m_leftClimbSide.setIdleMode(IdleMode.kBrake);
    m_rightClimbSide.setIdleMode(IdleMode.kBrake);
    m_rightClimbSide.follow(m_leftClimbSide, true);
  }

  public void setClimb(double power) {
    m_leftClimbSide.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
