// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

// import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
// import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Intake extends SubsystemBase {

  private final DoubleSolenoid m_solenoidLeader =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          IntakeConstants.kLeaderForward,
          IntakeConstants.kLeaderReverse);

  private final DoubleSolenoid m_solenoidFollower =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          IntakeConstants.kFollowerForward,
          IntakeConstants.kFollowerReverse);

  WPI_VictorSPX m_motor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorPort);

  /** Creates a new Intake. */
  public Intake() {}

  public void extendIntake() {
    m_solenoidLeader.set(kForward);
    m_solenoidFollower.set(kForward);
  }

  public void retractIntake() {
    m_solenoidLeader.set(kReverse);
    m_solenoidFollower.set(kReverse);
  }

  public void intakeBalls() {
    if (m_solenoidFollower.get() != kForward) {
      extendIntake();
    }
    m_motor.setVoltage(IntakeConstants.kIntakeVoltage);
  }

  public void outtakeBalls() {
    if (m_solenoidFollower.get() != kForward) {
      extendIntake();
    }
    m_motor.setVoltage(-IntakeConstants.kIntakeVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
