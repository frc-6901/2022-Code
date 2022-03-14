// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.util.MagEncoderUtil;

public class Drivetrain extends SubsystemBase {

  private WPI_TalonSRX m_leftSRX = new WPI_TalonSRX(DrivetrainConstants.kLeftSRXDrivePort);
  private WPI_VictorSPX m_leftSPX = new WPI_VictorSPX(DrivetrainConstants.kLeftSPXDrivePort);

  private WPI_TalonSRX m_rightSRX = new WPI_TalonSRX(DrivetrainConstants.kRightSRXDrivePort);
  private WPI_VictorSPX m_rightSPX = new WPI_VictorSPX(DrivetrainConstants.kRightSPXDrivePort);

  private TalonSRXSimCollection m_leftTalonSim = new TalonSRXSimCollection(m_leftSRX);
  private TalonSRXSimCollection m_rightTalonSim = new TalonSRXSimCollection(m_rightSRX);

  private WPI_PigeonIMU m_pigeon;

  BasePigeonSimCollection m_pigeonSim;

  private DifferentialDrive m_drive = new DifferentialDrive(m_leftSRX, m_rightSRX);

  private DifferentialDrivetrainSim m_drivetrainSim =
      new DifferentialDrivetrainSim(
          LinearSystemId.identifyDrivetrainSystem(
              DrivetrainConstants.kLinearKV,
              DrivetrainConstants.kLinearKA,
              DrivetrainConstants.kAngularKV,
              DrivetrainConstants.kAngularKA),
          DCMotor.getCIM(2),
          DrivetrainConstants.kCimToWheelGearing,
          DrivetrainConstants.kTrackwidth,
          Units.inchesToMeters(DrivetrainConstants.kWheelDiameterInches / 2),
          null);

  private Pose2d position;

  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d());
  private Field2d m_field = new Field2d();

  private PIDController m_leftController =
      new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
  private PIDController m_righController =
      new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
  private SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(
          DrivetrainConstants.kLinearKS,
          DrivetrainConstants.kLinearKV,
          DrivetrainConstants.kLinearKA);

  private TrajectoryConfig m_trajectoryConfig;

  /** Creates a new Drivetrain. */
  public Drivetrain(WPI_PigeonIMU pigeon) {
    m_leftSPX.follow(m_leftSRX);
    m_rightSPX.follow(m_rightSRX);
    m_rightSPX.setInverted(InvertType.FollowMaster);

    m_rightSRX.setInverted(InvertType.InvertMotorOutput);

    m_leftSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    m_rightSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    m_leftSRX.setSensorPhase(false);
    m_rightSRX.setSensorPhase(false);

    m_leftSRX.setNeutralMode(NeutralMode.Brake);
    m_rightSRX.setNeutralMode(NeutralMode.Brake);
    m_leftSPX.setNeutralMode(NeutralMode.Brake);
    m_rightSPX.setNeutralMode(NeutralMode.Brake);

    m_leftSRX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 1));
    m_rightSRX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 1));

    // Simulated pigeon can't be connected to a Talon
    if (RobotBase.isSimulation()) {
      m_pigeon = new WPI_PigeonIMU(1);
    } else {
      m_pigeon = pigeon;
    }

    m_pigeonSim = m_pigeon.getSimCollection();

    var voltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DrivetrainConstants.kLinearKS,
                DrivetrainConstants.kLinearKV,
                DrivetrainConstants.kLinearKA),
            DrivetrainConstants.kDriveKinematics,
            10);
    m_trajectoryConfig =
        new TrajectoryConfig(DrivetrainConstants.kAutoMaxSpeed, DrivetrainConstants.kAutoMaxAccel);

    m_trajectoryConfig.setKinematics(DrivetrainConstants.kDriveKinematics);
    m_trajectoryConfig.addConstraint(voltageConstraint);

    resetOdo();
    SmartDashboard.putData("Field", m_field);
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        MagEncoderUtil.nativeUnitsToVelocity(
            m_leftSRX.getSelectedSensorVelocity(), DrivetrainConstants.kWheelCircumferenceMeters),
        MagEncoderUtil.nativeUnitsToVelocity(
            m_rightSRX.getSelectedSensorVelocity(), DrivetrainConstants.kWheelCircumferenceMeters));
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public SimpleMotorFeedforward getSimpleMotorFeedforward() {
    return m_feedForward;
  }

  public PIDController getLeftPIDController() {
    return m_leftController;
  }

  public void setOutput(double leftVolt, double rightVolt) {
    m_leftSRX.setVoltage(leftVolt);
    m_rightSRX.setVoltage(rightVolt);
  }

  public Pose2d getPose() {
    return position;
  }

  public DifferentialDriveKinematics getKinematics() {
    return DrivetrainConstants.kDriveKinematics;
  }

  public PIDController getRightPIDController() {
    return m_righController;
  }

  public void resetOdo() {
    m_pigeon.setYaw(0.0);
    m_rightSRX.setSelectedSensorPosition(0.0);
    m_leftSRX.setSelectedSensorPosition(0.0);
  }

  public TrajectoryConfig getTrajectoryConfig() {
    return m_trajectoryConfig;
  }

  public Command getTrajectoryFollowerCommand(Trajectory traj) {
    RamseteController driveController = new RamseteController(2.0, 0.7);
    driveController.setEnabled(false);
    RamseteCommand command =
        new RamseteCommand(
            traj,
            this::getPose,
            driveController,
            this.getSimpleMotorFeedforward(),
            getKinematics(),
            this::getWheelSpeeds,
            getLeftPIDController(),
            getRightPIDController(),
            this::setOutput,
            this);

    return command.andThen(
        () -> this.setOutput(0, 0)); // run command and after the command stop the drivetrain
  }

  @Override
  public void simulationPeriodic() {
    m_leftTalonSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightTalonSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_drivetrainSim.setInputs(
        m_leftTalonSim.getMotorOutputLeadVoltage(), -m_rightTalonSim.getMotorOutputLeadVoltage());
    m_drivetrainSim.update(0.02);

    m_leftTalonSim.setQuadratureRawPosition(
        MagEncoderUtil.distanceToNativeUnits(
            m_drivetrainSim.getLeftPositionMeters(),
            DrivetrainConstants.kWheelCircumferenceMeters));
    m_leftTalonSim.setQuadratureVelocity(
        MagEncoderUtil.velocityToNativeUnits(
            m_drivetrainSim.getLeftVelocityMetersPerSecond(),
            DrivetrainConstants.kWheelCircumferenceMeters));
    m_rightTalonSim.setQuadratureRawPosition(
        MagEncoderUtil.distanceToNativeUnits(
            -m_drivetrainSim.getRightPositionMeters(),
            DrivetrainConstants.kWheelCircumferenceMeters));
    m_rightTalonSim.setQuadratureVelocity(
        MagEncoderUtil.velocityToNativeUnits(
            -m_drivetrainSim.getRightVelocityMetersPerSecond(),
            DrivetrainConstants.kWheelCircumferenceMeters));
    m_pigeonSim.setRawHeading(m_drivetrainSim.getHeading().getDegrees());
  }

  @Override
  public void periodic() {
    position =
        m_odometry.update(
            m_pigeon.getRotation2d(),
            MagEncoderUtil.nativeUnitsToDistance(
                m_leftSRX.getSelectedSensorPosition(),
                DrivetrainConstants.kWheelCircumferenceMeters),
            MagEncoderUtil.nativeUnitsToDistance(
                m_rightSRX.getSelectedSensorPosition(),
                DrivetrainConstants.kWheelCircumferenceMeters));

    SmartDashboard.putNumber("Gyro Leader Voltage", m_leftSRX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Left Follower Voltage", m_leftSPX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Leader Voltage", m_rightSRX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Follower Voltage", m_rightSPX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Gyro Degrees", m_pigeon.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Left Encoder Values", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Right Encoder Values", getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("Left Vel Setpoint", m_leftController.getSetpoint());
    SmartDashboard.putNumber("Right Vel Setpoint", m_righController.getSetpoint());
    SmartDashboard.putNumber(
        "Left Velocity",
        MagEncoderUtil.nativeUnitsToVelocity(
            m_leftSRX.getSelectedSensorVelocity(), DrivetrainConstants.kWheelCircumferenceMeters));
    SmartDashboard.putNumber(
        "Right Velocity",
        MagEncoderUtil.nativeUnitsToVelocity(
            m_rightSRX.getSelectedSensorVelocity(), DrivetrainConstants.kWheelCircumferenceMeters));
    SmartDashboard.putNumber(
        "Left Encoder Distance",
        MagEncoderUtil.nativeUnitsToDistance(
            m_leftSRX.getSelectedSensorPosition(), DrivetrainConstants.kWheelCircumferenceMeters));
    SmartDashboard.putNumber(
        "Right Encoder Distance",
        MagEncoderUtil.nativeUnitsToDistance(
            m_rightSRX.getSelectedSensorPosition(), DrivetrainConstants.kWheelCircumferenceMeters));
    SmartDashboard.putNumber("Left Encoder Distance Raw", m_leftSRX.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder Distance Raw", m_rightSRX.getSelectedSensorPosition());

    SmartDashboard.putNumber("X ", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y ", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Rot ", m_odometry.getPoseMeters().getRotation().getDegrees());
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }
}
