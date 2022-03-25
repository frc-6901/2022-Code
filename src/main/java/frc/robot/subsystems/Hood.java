// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.MagEncoderUtil;

public class Hood extends SubsystemBase {
  private WPI_TalonSRX m_hoodMotor = new WPI_TalonSRX(HoodConstants.kHoodSRXPort);
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(HoodConstants.kMaxVelocity, HoodConstants.kMaxAccel);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private HoodState m_state = HoodState.kOpenLoop;
  private double m_openLoopPower = 0.0;

  private ArmFeedforward m_feedforward =
      new ArmFeedforward(HoodConstants.kS, HoodConstants.kCos, HoodConstants.kV, HoodConstants.kA);
  private PIDController m_controller = new PIDController(HoodConstants.kP, 0, HoodConstants.kD);
  private SingleJointedArmSim m_hoodSim =
      new SingleJointedArmSim(
          LinearSystemId.identifyPositionSystem(HoodConstants.kV, HoodConstants.kA),
          DCMotor.getVex775Pro(1),
          HoodConstants.kHoodGearing,
          Units.inchesToMeters(HoodConstants.kHoodCMLengthInches),
          Units.degreesToRadians(HoodConstants.kMinAngleDegrees),
          Units.degreesToRadians(HoodConstants.kMaxAngleDegrees),
          Units.lbsToKilograms(HoodConstants.kHoodMassPounds),
          true);

  private TalonSRXSimCollection m_hoodTalonSim = new TalonSRXSimCollection(m_hoodMotor);

  private Mechanism2d m_hoodBase = new Mechanism2d(60, 60);
  private MechanismRoot2d m_hoodRoot = m_hoodBase.getRoot("Hood", 30, 30);
  private MechanismLigament2d m_hoodDrawing =
      m_hoodRoot.append(new MechanismLigament2d("Hood Base", 15, 0));
  private MechanismLigament2d m_hoodEndDrawing =
      m_hoodDrawing.append(new MechanismLigament2d("Hood End", 5, 120));

  private double m_targetAngle = 0.0;

  public enum HoodState {
    kOpenLoop,
    kClosedLoop
  }

  /** Creates a new Hood. */
  public Hood() {
    m_hoodMotor.configFactoryDefault();
    m_hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_hoodMotor.setSensorPhase(false);
    m_hoodMotor.setInverted(InvertType.InvertMotorOutput);
    m_hoodMotor.configForwardSoftLimitThreshold(
        MagEncoderUtil.distanceToNativeUnits(30, 360, HoodConstants.kHoodGearing));
    m_hoodMotor.configForwardSoftLimitEnable(true);
    m_hoodMotor.configReverseSoftLimitThreshold(
        MagEncoderUtil.distanceToNativeUnits(0, 360, HoodConstants.kHoodGearing));
    m_hoodMotor.configReverseSoftLimitEnable(true);
    m_hoodMotor.setSelectedSensorPosition(0.0);
    // m_hoodMotor.config_kP(0, HoodConstants.kP);
  }

  public void set(double percentPower) {
    m_state = HoodState.kOpenLoop;
    m_openLoopPower = percentPower;
  }

  public double getPositionDegrees() {
    return MagEncoderUtil.nativeUnitsToDistance(
        m_hoodMotor.getSelectedSensorPosition(), 360, HoodConstants.kHoodGearing);
  }

  public double getVelocityDegreesPerSec() {
    return MagEncoderUtil.nativeUnitsToVelocity(
        m_hoodMotor.getSelectedSensorVelocity(), 360, HoodConstants.kHoodGearing);
  }

  public void setTargetPosition(double degrees) {
    m_targetAngle = degrees;
    m_state = HoodState.kClosedLoop;
  }

  public boolean atTargetPosition() {
    return Math.abs(m_targetAngle - getPositionDegrees()) < HoodConstants.kMaxDegreeError;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood Position", getPositionDegrees());
    SmartDashboard.putNumber("Target Position", m_targetAngle);
    SmartDashboard.putNumber("Hood Position Rad", Units.degreesToRadians(getPositionDegrees()));

    if (m_state == HoodState.kClosedLoop) {
      SmartDashboard.putString("Hood State", "CLOSED");
      m_openLoopPower = 0.0;
      double output = m_controller.calculate(getPositionDegrees(), m_targetAngle);
      SmartDashboard.putNumber("Output", output);
      m_hoodMotor.setVoltage(output);
    } else {
      SmartDashboard.putString("Hood State", "OPEN");
      if (Math.abs(m_openLoopPower) > 0.25) {
        m_hoodMotor.set((m_openLoopPower > 0 ? 1 : -1) * HoodConstants.kOpenLoopPercentPower);
      } else {
        m_hoodMotor.set(0);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    m_hoodTalonSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_hoodSim.setInputVoltage(m_hoodTalonSim.getMotorOutputLeadVoltage());
    m_hoodSim.update(Constants.kDt);

    m_hoodTalonSim.setQuadratureRawPosition(
        MagEncoderUtil.distanceToNativeUnits(
            Units.radiansToDegrees(m_hoodSim.getAngleRads()), 360, HoodConstants.kHoodGearing));
    m_hoodTalonSim.setQuadratureVelocity(
        MagEncoderUtil.velocityToNativeUnits(
            Units.radiansToDegrees(m_hoodSim.getVelocityRadPerSec()),
            360,
            HoodConstants.kHoodGearing));

    m_hoodDrawing.setAngle(getPositionDegrees());
    SmartDashboard.putData("Hood Mech2d", m_hoodBase);

    SmartDashboard.putNumber("Hood Volts", m_hoodTalonSim.getMotorOutputLeadVoltage());
  }
}
