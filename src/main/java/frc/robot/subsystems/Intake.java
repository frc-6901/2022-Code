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

public class Intake extends SubsystemBase {

  private final DoubleSolenoid m_intakeSolenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          IntakeConstants.kLeftForward,
          IntakeConstants.kRightForward);
      
  WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorPort);

  /** Creates a new Intake. */
  public Intake() {}

  public void extendIntake() {
    m_intakeSolenoid.set(kForward);
  }

  public void retractIntake() {
    m_intakeSolenoid.set(kReverse);
  }

  public void noBalls() {
    m_intakeMotor.setVoltage(0);
  }

  public void intakeBalls() {
    if (m_intakeSolenoid.get() != kForward) {
      extendIntake();
    }
    m_intakeMotor.setVoltage(-IntakeConstants.kIntakeVoltage);
  }

  public void outtakeBalls() {
    if (m_intakeSolenoid.get() != kForward) {
      extendIntake();
    }
    m_intakeMotor.setVoltage(IntakeConstants.kIntakeVoltage);
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
