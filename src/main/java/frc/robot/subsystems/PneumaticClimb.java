// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticClimbConstants;

public class PneumaticClimb extends SubsystemBase {

  private final DoubleSolenoid m_climbSolenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          PneumaticClimbConstants.kClimbSolenoidPorts[0],
          PneumaticClimbConstants.kClimbSolenoidPorts[1]);

  /** Creates a new ExampleSubsystem. */
  public PneumaticClimb() {
    m_climbSolenoid.set(kOff);
  }

  // Extends pneumatic climb
  public void extendClimb() {
    m_climbSolenoid.set(kForward);
  }

  // Retracts pneumatic climb
  public void retractClimb() {
    m_climbSolenoid.set(kReverse);
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
