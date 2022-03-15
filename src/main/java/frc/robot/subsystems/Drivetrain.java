// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonSRX m_leftSRX = new WPI_TalonSRX(DrivetrainConstants.kLeftSRXDrivePort);
  private WPI_VictorSPX m_leftSPX = new WPI_VictorSPX(DrivetrainConstants.kLeftSPXDrivePort);

  private WPI_TalonSRX m_rightSRX = new WPI_TalonSRX(DrivetrainConstants.kRightSRXDrivePort);
  private WPI_VictorSPX m_rightSPX = new WPI_VictorSPX(DrivetrainConstants.kRightSPXDrivePort);

  private PigeonIMU m_pigeon;

  private DifferentialDrive m_drive = new DifferentialDrive(m_leftSRX, m_rightSRX);

  /** Creates a new Drivetrain. */
  public Drivetrain(PigeonIMU pigeon) {
    m_leftSPX.follow(m_leftSRX);
    m_rightSRX.setInverted(true);
    m_rightSPX.setInverted(true);
    m_rightSPX.follow(m_rightSRX);

    m_leftSRX.setNeutralMode(NeutralMode.Brake);
    m_leftSPX.setNeutralMode(NeutralMode.Brake);
    m_rightSRX.setNeutralMode(NeutralMode.Brake);
    m_rightSPX.setNeutralMode(NeutralMode.Brake);

    m_pigeon = pigeon;
  }

  /**
   * Drives the robot in an arcade drive
   *
   * @param forward Forward value as a percent output: [-1, 1]
   * @param rotate Rotate value as a percent output: [-1, 1]
   */
  public void drive(double forward, double rotate) {
    m_drive.arcadeDrive(forward, rotate);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Leader Voltage", m_leftSRX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Left Follower Voltage", m_leftSPX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Leader Voltage", m_rightSRX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Follower Voltage", m_rightSPX.getMotorOutputVoltage());
  }
}
