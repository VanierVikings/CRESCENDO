// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.RobotContainer;
//import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  public AHRS navX = new AHRS(Port.kMXP);
  public CANSparkMax frontLeftMotor = new CANSparkMax(DriverConstants.leftFrontId, MotorType.kBrushless);
  public CANSparkMax backLeftMotor = new CANSparkMax(DriverConstants.leftRearId, MotorType.kBrushless);
  public CANSparkMax frontRightMotor = new CANSparkMax(DriverConstants.rightFrontId, MotorType.kBrushless);
  public CANSparkMax backRightMotor = new CANSparkMax(DriverConstants.rightRearId, MotorType.kBrushless);
  SparkPIDController frontLeftPID = frontLeftMotor.getPIDController();
  SparkPIDController backLeftPID = backLeftMotor.getPIDController();
  SparkPIDController frontRightPID = frontRightMotor.getPIDController();
  SparkPIDController backRightPID = backRightMotor.getPIDController();
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriverConstants.kS, DriverConstants.kV, DriverConstants.kA);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriverConstants.trackWidth);

  public RelativeEncoder leftEncoder = frontLeftMotor.getEncoder();
  public RelativeEncoder rightEncoder = frontRightMotor.getEncoder();
  private final PIDController m_leftPIDController = new PIDController(DriverConstants.kPL, DriverConstants.kI, DriverConstants.kD);
  private final PIDController m_rightPIDController = new PIDController(DriverConstants.kPR, DriverConstants.kI, DriverConstants.kD);
  public DifferentialDrive difDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
  Pose2d m_pose = new Pose2d(0, 0, navX.getRotation2d());
  public DifferentialDriveOdometry m_odometry;

  public Rotation2d getRotation2d() {

    return navX.getRotation2d();
  }
  public void pidControllerSetUp(){
    // putNumber("FF:",frontLeftPID.getFF());
    // SmartDasSmartDashboard.hboard.putNumber("P", frontLeftPID.getP());
    // SmartDashboard.putNumber("I", frontLeftPID.getI());
    // SmartDashboard.putNumber("D", frontLeftPID.getD());
    frontLeftPID.setFF(0.0002,0);
    frontLeftPID.setP(0.,0);
    frontLeftPID.setI(0,0);
    frontLeftPID.setD(0,0);

    backLeftPID.setFF(0.0002,0);
    backLeftPID.setP(0.,0);
    backLeftPID.setI(0,0);
    backLeftPID.setD(0,0);

    frontRightPID.setFF(0.0002,0);
    frontRightPID.setP(0,0);
    frontRightPID.setI(0,0);
    frontRightPID.setD(0,0);

    backRightPID.setFF(0.0002,0);
    backRightPID.setP(0,0);
    backRightPID.setI(0,0);
    backRightPID.setD(0,0);
  }
  public void driveWithReference(double forward, double turn){
    int maxRPM = 5600;
    double speedLeft = forward + turn;
    double speedRight = forward-turn;
    if(speedLeft>1){
      speedLeft = 1;
    }
    else if(speedLeft<-1){
      speedLeft = -1;
    }
    if(speedRight>1){
      speedRight = 1;
    }
    else if(speedRight<-1){
      speedRight = -1;
    }
    frontLeftPID.setReference(speedLeft*maxRPM, ControlType.kVelocity,0);
    frontRightPID.setReference(speedRight*maxRPM, ControlType.kVelocity,0);
    backLeftPID.setReference(speedLeft*maxRPM, ControlType.kVelocity,0);
    backRightPID.setReference(speedRight*maxRPM, ControlType.kVelocity,0);

  }


  public double getLeftWheel() {
    return getLeftEncoderVelocity();
  }

  public double getRightWheel() {
    return getRightEncoderVelocity();
  }

  public void resetGyro() {
    navX.reset();
  }

  public double getGyroYaw() {
    return navX.getYaw() % 360;
  }

  public double getGyroPitch() {
    // 0.31 is offset
    return navX.getPitch() % 360;
  }

  public double getGyroRoll() {
    return navX.getRoll();
  }

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    
    difDrive.setSafetyEnabled(false);

    frontLeftMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    rightEncoder.setPositionConversionFactor(DriverConstants.positionConversionFactor);
    leftEncoder.setPositionConversionFactor(-DriverConstants.positionConversionFactor);
    rightEncoder.setVelocityConversionFactor(DriverConstants.velocityConversionFactor / 60);
    leftEncoder.setVelocityConversionFactor(-DriverConstants.velocityConversionFactor / 60);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    // frontLeftMotor.enableVoltageCompensation(10);
    // frontRightMotor.enableVoltageCompensation(10);

    navX.reset();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), 0, 0);
    
    setBrakeMode();
    AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassiSpeeds, // Current ChassisSpeeds supplier
            this::driveChassis, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  public double getAverageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  public void setBrakeMode() {
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    backLeftMotor.setIdleMode(IdleMode.kCoast);
    frontLeftMotor.setIdleMode(IdleMode.kCoast);
    frontRightMotor.setIdleMode(IdleMode.kCoast);
    backRightMotor.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void drive(double forward, double rotate) {
    if(frontLeftMotor.getEncoder().getVelocity()<2000){
      frontLeftMotor.setSmartCurrentLimit(60);
      backLeftMotor.setSmartCurrentLimit(60);
    }
    else{
            frontLeftMotor.setSmartCurrentLimit(80);
            backLeftMotor.setSmartCurrentLimit(80);
    }
        if(frontRightMotor.getEncoder().getVelocity()<2000){
      frontRightMotor.setSmartCurrentLimit(40);
      backRightMotor.setSmartCurrentLimit(40);
    }
    else{
            frontRightMotor.setSmartCurrentLimit(60);
            backRightMotor.setSmartCurrentLimit(60);
    }
    difDrive.arcadeDrive(forward, rotate);
  }

  public void nullRampRates() {
    backLeftMotor.setOpenLoopRampRate(0);
    frontLeftMotor.setOpenLoopRampRate(0);
    backRightMotor.setOpenLoopRampRate(0);
    frontRightMotor.setOpenLoopRampRate(0);
    backLeftMotor.setClosedLoopRampRate(0);
    frontLeftMotor.setClosedLoopRampRate(0);
    backRightMotor.setClosedLoopRampRate(0);
    frontRightMotor.setClosedLoopRampRate(0);

  }

  public double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return leftEncoder.getVelocity();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(navX.getRotation2d(),
        new DifferentialDriveWheelPositions(getLeftEncoderPosition(), getRightEncoderPosition()), pose);
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void setCurrentLimits(int limit) {
    frontLeftMotor.setSmartCurrentLimit(limit);
    frontRightMotor.setSmartCurrentLimit(limit);
    backLeftMotor.setSmartCurrentLimit(limit);
    backRightMotor.setSmartCurrentLimit(limit);
  }

  
  

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(rightVolts);
    difDrive.feed();
  }
  public void voltageArcadeDrive(double forward, double turn){
    
    double leftEffort = forward+turn;
    double rightEffort = forward-turn;
    if(leftEffort>1)
    leftEffort=1;
    if(leftEffort<-1)
    leftEffort=-1;
    
    if(rightEffort>1)
    rightEffort=1;
    if(rightEffort<-1)
    rightEffort=-1;
    frontLeftMotor.setVoltage(frontLeftMotor.getBusVoltage()*leftEffort*1.2);
    backLeftMotor.setVoltage(backLeftMotor.getBusVoltage()*leftEffort*1.2);
    frontRightMotor.setVoltage(frontRightMotor.getBusVoltage()*rightEffort*1.2);
    backRightMotor.setVoltage(backRightMotor.getBusVoltage()*rightEffort*1.2);
  }
    public void voltageTankDrive(double left, double right){
    
    
    frontLeftMotor.setVoltage(frontLeftMotor.getBusVoltage()*left*1.2);
    backLeftMotor.setVoltage(backLeftMotor.getBusVoltage()*left*1.2);
    frontRightMotor.setVoltage(frontRightMotor.getBusVoltage()*right*1.2);
    backRightMotor.setVoltage(backRightMotor.getBusVoltage()*right*1.2);
  }



  public double getAverageRPM() {
    return (Math.abs(getLeftWheel()) + Math.abs(getRightWheel())) * 30
        / DriverConstants.positionConversionFactor;
  }

  public double getAverageCurrent() {
    return (frontLeftMotor.getOutputCurrent() + frontRightMotor.getOutputCurrent()) / 2;
  }

  @Override
  public void periodic() {
    // if (driverControlled) {
    // adjustCurrentLimit();
    // }
    SmartDashboard.putNumber("Gyro Angle",m_odometry.getPoseMeters().getRotation().getDegrees());
    m_odometry.update(navX.getRotation2d(), getLeftEncoderPosition(),
        getRightEncoderPosition());
        // SmartDashboard.putNumber("Left Wheel Velocity", getLeftEncoderVelocity());
        // SmartDashboard.putNumber("Right Wheel Velocity", getRightEncoderVelocity());
  }

  public Trajectory getTrajectory(String filePath) {

    try {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("paths/" + RobotContainer.m_alliance.getSelected() + filePath + ".wpilib.json");
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);
        return trajectory;
 
    } catch (IOException ex) {
      // DriverStation.reportError("Unable to open trajectory: " + ex.getMessage(), ex.getStackTrace());
    }
    
    try {
      return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/" + RobotContainer.m_alliance.getSelected() + filePath + ".wpilib.json"));
    } catch (IOException e) {
      e.printStackTrace();
    }
    try {
      return  TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/BlueShortTaxi.wpilib.json"));
    } catch (Exception e) {
      return null;    
}

  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
   public ChassisSpeeds getChassiSpeeds() {
    return new ChassisSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity(), navX.getAngle());
  }
  public void driveChassis(ChassisSpeeds speed){
    setSpeeds(kinematics.toWheelSpeeds(speed));
  }
   public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
    frontLeftMotor.set(leftOutput + leftFeedforward/RobotController.getBatteryVoltage());
    frontRightMotor.set(rightOutput + rightFeedforward/RobotController.getBatteryVoltage());
  }

  
}