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
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

  private WPI_TalonSRX m_leftSRX = new WPI_TalonSRX(DrivetrainConstants.kLeftSRXDrivePort);
  private WPI_VictorSPX m_leftSPX = new WPI_VictorSPX(DrivetrainConstants.kLeftSPXDrivePort);

  private WPI_TalonSRX m_rightSRX = new WPI_TalonSRX(DrivetrainConstants.kRightSRXDrivePort);
  private WPI_VictorSPX m_rightSPX = new WPI_VictorSPX(DrivetrainConstants.kRightSPXDrivePort);

  private TalonSRXSimCollection m_leftTalonSim = new TalonSRXSimCollection(m_leftSRX);
  private TalonSRXSimCollection m_rightTalonSim = new TalonSRXSimCollection(m_rightSRX);

  private WPI_PigeonIMU m_pigeon;

  BasePigeonSimCollection m_pigeonSim;

  private DifferentialDriveKinematics diffDrive =
      new DifferentialDriveKinematics(DrivetrainConstants.kTrackwidth);
  private DifferentialDrive m_drive = new DifferentialDrive(m_leftSRX, m_rightSRX);

  private DifferentialDrivetrainSim m_drivetrainSim = new DifferentialDrivetrainSim(LinearSystemId.identifyDrivetrainSystem(DrivetrainConstants.kLinearKV, DrivetrainConstants.kLinearKA, DrivetrainConstants.kAngularKV, DrivetrainConstants.kAngularKA), DCMotor.getCIM(2), DrivetrainConstants.kCimToWheelGearing, DrivetrainConstants.kTrackwidth, Units.inchesToMeters(DrivetrainConstants.kWheelDiameterInches / 2), VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  private Pose2d position;

  private DifferentialDriveOdometry m_Odometry = new DifferentialDriveOdometry(new Rotation2d());

  private PIDController m_leftController =
      new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
  private PIDController m_righController =
      new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
  private SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(
          DrivetrainConstants.kLinearKS,
          DrivetrainConstants.kLinearKV,
          DrivetrainConstants.kLinearKA);

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
        ticksToDistance(m_leftSRX.getSelectedSensorVelocity()) / 0.1,
        ticksToDistance(m_rightSRX.getSelectedSensorVelocity()) / 0.1);
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public SimpleMotorFeedforward gSimpleMotorFeedforward() {
    return m_feedForward;
  }

  public PIDController getLeftPIDController() {
    return m_leftController;
  }

  public static double ticksToDistance(double ticks) {
    return ticks * 1.0 / 4096.0 * 6 * Math.PI * 0.0254;
  }

  public void setOutput(double leftVolt, double rightVolt) {
    m_leftSRX.setVoltage(leftVolt);
    m_rightSRX.setVoltage(rightVolt);
  }

  public Pose2d getPose() {
    return position;
  }

  public DifferentialDriveKinematics getKinematics() {
    return diffDrive;
  }

  public PIDController getRightPIDController() {
    return m_righController;
  }
  // // These represent our regular encoder objects, which we would
  // // create to use on a real robot.
  public Field2d field = new Field2d();
  // public Encoder m_leftEncoder = new Encoder(0, 1);
  // public Encoder m_rightEncoder = new Encoder(2, 3);

  // // These are our EncoderSim objects, which we will only use in
  // // simulation. However, you do not need to comment out these
  // // declarations when you are deploying code to the roboRIO.
  // public EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  // public EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  // // Create our gyro object like we would on a real robot.
  // public AnalogGyro m_gyro = new AnalogGyro(1);

  // // Create the simulated gyro object, used for setting the gyro
  // // angle. Like EncoderSim, this does not need to be commented out
  // // when deploying code to the roboRIO.
  // public AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  // DifferentialDrivetrainSim m_driveSim =
  //     new DifferentialDrivetrainSim(
  //         DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
  //         7.29, // 7.29:1 gearing reduction.
  //         7.5, // MOI of 7.5 kg m^2 (from CAD model).
  //         60.0, // The mass of the robot is 60 kg.
  //         Units.inchesToMeters(3), // The robot uses 3" radius wheels.
  //         0.7112, // The track width is 0.7112 meters.

  //         // The standard deviations for measurement noise:
  //         // x and y:          0.001 m
  //         // heading:          0.001 rad
  //         // l and r velocity: 0.1   m/s
  //         // l and r position: 0.005 m
  //         VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  // public DifferentialDriveOdometry odom = new DifferentialDriveOdometry(m_gyro.getRotation2d());

  /** Creates a new Drivetrain. */
  public Drivetrain(WPI_PigeonIMU pigeon) {
    m_leftSRX.setNeutralMode(NeutralMode.Brake);
    m_leftSPX.setNeutralMode(NeutralMode.Brake);
    m_rightSRX.setNeutralMode(NeutralMode.Brake);
    m_rightSPX.setNeutralMode(NeutralMode.Brake);

    m_pigeon = pigeon;
    m_leftSPX.follow(m_leftSRX);
    m_rightSPX.follow(m_rightSRX);
    m_rightSPX.setInverted(InvertType.FollowMaster);

    m_rightSRX.setInverted(InvertType.InvertMotorOutput);
    
    

    m_leftSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    m_rightSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    m_leftSRX.setSensorPhase(true);
    m_rightSRX.setSensorPhase(true);

    m_leftSRX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 1));
    m_rightSRX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 1));

    m_pigeonSim = m_pigeon.getSimCollection();
    resetOdo();
  }

  public void resetOdo() {
    m_pigeon.setYaw(0.0);
    m_rightSRX.setSelectedSensorPosition(0.0);
    m_leftSRX.setSelectedSensorPosition(0.0);
  }

  @Override
  public void simulationPeriodic() {
    m_leftTalonSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightTalonSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_leftTalonSim.setQuadratureRawPosition(
                    distanceToNativeUnits(
                      m_drivetrainSim.getLeftPositionMeters()
                    ));
                    m_leftTalonSim.setQuadratureVelocity(
                    velocityToNativeUnits(
                      m_drivetrainSim.getLeftVelocityMetersPerSecond()
                    ));
                    m_rightTalonSim.setQuadratureRawPosition(
                    distanceToNativeUnits(
                        -m_drivetrainSim.getRightPositionMeters()
                    ));
                    m_rightTalonSim.setQuadratureVelocity(
                    velocityToNativeUnits(
                        -m_drivetrainSim.getRightVelocityMetersPerSecond()
                    ));
    m_pigeonSim.setRawHeading(m_drivetrainSim.getHeading().getDegrees());

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Gyro Leader Voltage", m_leftSRX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Left Follower Voltage", m_leftSPX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Leader Voltage", m_rightSRX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Follower Voltage", m_rightSPX.getMotorOutputVoltage());
    SmartDashboard.putNumber("Gyro Degrees", m_pigeon.getYaw());
    SmartDashboard.putNumber("Left Encoder Values", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Right Encoder Values", getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("Left Vel Setpoint", m_leftController.getSetpoint());
    SmartDashboard.putNumber("Right Vel Setpoint", m_righController.getSetpoint());
    SmartDashboard.putNumber(
        "Left Encoder Distance", ticksToDistance(m_leftSRX.getSelectedSensorPosition()));
    SmartDashboard.putNumber(
        "Right Encoder Distance", ticksToDistance(m_rightSRX.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Left Encoder Distance Raw", m_leftSRX.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder Distance Raw", m_rightSRX.getSelectedSensorPosition());

    position =
        m_Odometry.update(
            m_pigeon.getRotation2d(),
            ticksToDistance(m_leftSRX.getSelectedSensorPosition()),
            ticksToDistance(m_rightSRX.getSelectedSensorPosition()));
    SmartDashboard.putNumber("X ", m_Odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y ", m_Odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Rot ", m_Odometry.getPoseMeters().getRotation().getDegrees());
  }
}
