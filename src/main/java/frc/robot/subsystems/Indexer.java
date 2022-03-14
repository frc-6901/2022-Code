// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private DigitalInput m_ballSensor = new DigitalInput(IndexerConstants.kProximitySensorPort);
  private WPI_TalonSRX m_ballElevatorMotor = new WPI_TalonSRX(IndexerConstants.kBallElevatorPort);
  private WPI_VictorSPX m_indexerMotor = new WPI_VictorSPX(IndexerConstants.kIndexerPort);

  private WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(m_ballElevatorMotor);

  private IndexerState m_indexerState = IndexerState.kPassive;

  /** Represents the different states of the indexer */
  public enum IndexerState {
    /** Passive state moves the indexer until a ball is detected */
    kPassive,
    /** Reverse state moves the ball out the indexer */
    kReverse,
    /** Reeding state is when the ball is being fed into the shooter */
    kFeeding,
    /** When the indexer is stopped */
    kStopped
  }
  /** Creates a new Indexer. */
  public Indexer() {
    m_ballElevatorMotor.configFactoryDefault();
    m_indexerMotor.configFactoryDefault();
  }

  /**
   * Get the pigeon hooked to the indexer Talon SRX
   *
   * @return Pigeon object hooked to the Talon SRX
   */
  public WPI_PigeonIMU getPigeon() {
    return m_pigeon;
  }

  /**
   * Method to determine if the proximity sensor detected the ball
   *
   * @return True if the ball is detected, False otherwise
   */
  public boolean ballDetected() {
    return !m_ballSensor.get();
  }

  /** Runs a state machine for the indexer to set the motors to their proper powers. */
  private void indexerStateMachine() {
    double ballElevatorPower = 0.0;
    double indexerPower = 0.0;
    if (m_indexerState == IndexerState.kPassive) {
      if (ballDetected()) {
        ballElevatorPower = 0.0;
        indexerPower = 0.0;
      } else {
        ballElevatorPower = IndexerConstants.kPassivePower;
        indexerPower = IndexerConstants.kIndexerPower;
      }
    } else if (m_indexerState == IndexerState.kFeeding) {
      ballElevatorPower = IndexerConstants.kFeedingPower;
      indexerPower = IndexerConstants.kIndexerPower;
    } else if (m_indexerState == IndexerState.kReverse) {
      ballElevatorPower = -IndexerConstants.kFeedingPower;
      indexerPower = -IndexerConstants.kIndexerPower;
    } else {
      ballElevatorPower = 0.0;
      indexerPower = 0.0;
    }
    m_ballElevatorMotor.setVoltage(ballElevatorPower);
    m_indexerMotor.setVoltage(-indexerPower);
  }

  /**
   * Sets the state of the indexer state machine
   *
   * @param state The desired state
   */
  public void setState(IndexerState state) {
    m_indexerState = state;
  }

  @Override
  public void periodic() {
    indexerStateMachine();
    SmartDashboard.putBoolean("Ball Detected", ballDetected());
  }
}
