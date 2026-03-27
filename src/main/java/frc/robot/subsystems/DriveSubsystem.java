// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class DriveSubsystem extends SubsystemBase {

  private final DifferentialDriveOdometry odometry;
  private final AHRS gyro;
  private final SparkMax leftMotor;
  private final SparkMax leftMotorFollower;
  private final SparkMax rightMotor;
  private final SparkMax rightMotorFollower;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final SparkClosedLoopController leftController;
  private final SparkClosedLoopController rightController;
  private final DifferentialDriveKinematics kinematics;
  private RobotConfig config;
  public DifferentialDrive robotDrive;
  private SparkMaxConfig globalConfig;
  private SparkMaxConfig rightConfig;
  private SparkMaxConfig leftFollowerConfig;
  private SparkMaxConfig rightFollowerConfig;
  private final Field2d m_field;
  public DriveSubsystem() {
    m_field = new Field2d();

    kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.trackWidthMeters);

    // navX Micro using usb

    gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k100Hz);

    // All other subsystem initialization
    leftMotor = new SparkMax(Constants.DriveConstants.leftMotor, MotorType.kBrushless);
    leftMotorFollower = new SparkMax(Constants.DriveConstants.leftFollower, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.DriveConstants.rightMotor, MotorType.kBrushless);
    rightMotorFollower = new SparkMax(Constants.DriveConstants.rightFollower, MotorType.kBrushless);

    globalConfig = new SparkMaxConfig();
    rightConfig = new SparkMaxConfig();
    leftFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig = new SparkMaxConfig();

    setConfigs();
    applyConfigs();
    // Leaders now have the through bore encoders
    leftEncoder = leftMotor.getAlternateEncoder();
    rightEncoder = rightMotor.getAlternateEncoder();
    leftController = leftMotor.getClosedLoopController();
    rightController = rightMotor.getClosedLoopController();

    robotDrive = new DifferentialDrive(leftMotor, rightMotor);
    robotDrive.setSafetyEnabled(false);
    robotDrive.setDeadband(0.04);

    odometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry
        this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPLTVController(
            0.02), // PPLTVController is the path following controller for differential drive
        config, // The robot configuration
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
     SmartDashboard.putData("Field", m_field);
     // Show PathPlanner path and target pose on the field too
      PathPlannerLogging.setLogCurrentPoseCallback((pose) -> m_field.setRobotPose(pose));
      PathPlannerLogging.setLogTargetPoseCallback((pose) -> 
          m_field.getObject("target pose").setPose(pose));
      PathPlannerLogging.setLogActivePathCallback((poses) -> 
          m_field.getObject("path").setPoses(poses));

          // Wheel radius characterization inputs/outputs
      SmartDashboard.putNumber("WheelChar/ActualDistanceMeters", 0.0);
      SmartDashboard.putNumber("WheelChar/StartLeft", 0.0);
      SmartDashboard.putNumber("WheelChar/StartRight", 0.0);
      SmartDashboard.putString("WheelChar/Status", "Ready");
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    System.out.println(pose);
    odometry.resetPosition(gyro.getRotation2d(), getCurrentPositions(), pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    DifferentialDriveWheelSpeeds currentSpeeds =
        new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity(), rightEncoder.getVelocity());
    return kinematics.toChassisSpeeds(currentSpeeds);
  }

  public DifferentialDriveWheelPositions getCurrentPositions() {
    DifferentialDriveWheelPositions positions =
        new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition());
    return positions;
  }

  /**
   * Method to Apply the configuration to the SPARKs.
   *
   * <p>kResetSafeParameters is used to get the SPARK MAX to a known state. This is useful in case
   * the SPARK MAX is replaced.
   *
   * <p>kPersistParameters is used to ensure the configuration is not lost when the SPARK MAX loses
   * power. This is useful for power cycles that may occur mid-operation.
   */
  private void applyConfigs() {
    leftMotor.configure(
        globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotorFollower.configure(
        leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotorFollower.configure(
        rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Set parameters that will apply to all SPARKs. We will also use this as the left leader config.
   */
  private void setConfigs() {
    globalConfig
        .smartCurrentLimit(46)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.4)
        .closedLoopRampRate(0.25)
        .voltageCompensation(12.0);

    // Built-in NEO encoder config (motor shaft, includes gear ratio) — kept in case we revert
    globalConfig
        .encoder
        .velocityConversionFactor(Constants.DriveConstants.velocityConversionFactor)
        .positionConversionFactor(Constants.DriveConstants.positionConversionFactor);

    // Through bore alternate encoder (output shaft, no gear ratio)
    globalConfig.alternateEncoder
        .setSparkMaxDataPortConfig()
        .countsPerRevolution(8192)
        .measurementPeriod(25)
        .averageDepth(8)
        .positionConversionFactor(Constants.DriveConstants.wheelCircumference)
        .velocityConversionFactor(Constants.DriveConstants.wheelCircumference / 60.0)
        .inverted(true);

    // Slot 0 — teleop: FF + kP for veering correction
    // kP in duty cycle (REV PID output is duty cycle) — already /12 in Constants
    // kS/kV in volts per REV docs
    globalConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(Constants.DriveConstants.kP, 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .feedForward
            .kS(Constants.DriveConstants.kS, ClosedLoopSlot.kSlot0)
            .kV(Constants.DriveConstants.kV, ClosedLoopSlot.kSlot0);

    // Slot 1 — auto: FF only, kP = 0, let LTV controller handle pose correction
    globalConfig.closedLoop
        .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1)
        .feedForward
            .kS(Constants.DriveConstants.kS, ClosedLoopSlot.kSlot1)
            .kV(Constants.DriveConstants.kV, ClosedLoopSlot.kSlot1);

    globalConfig
        .signals
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(5);

    // Apply the global config and invert since it is on the opposite side
    // Override alternateEncoder inversion — right encoder is not inverted
    rightConfig.apply(globalConfig).inverted(true);
    rightConfig.alternateEncoder.inverted(false);

    leftFollowerConfig
        .smartCurrentLimit(46)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.4)
        .closedLoopRampRate(0.25)
        .voltageCompensation(12.0);

    leftFollowerConfig
        .signals
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(5);
    leftFollowerConfig.follow(leftMotor);

    rightFollowerConfig
        .smartCurrentLimit(46)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.4)
        .closedLoopRampRate(0.25)
        .voltageCompensation(12.0);

    rightFollowerConfig
        .signals
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(5);
    rightFollowerConfig.follow(rightMotor);
  }

    public void stop() {
    robotDrive.arcadeDrive(0, 0);
  }
  /** drive method for pathplanner — slot 1, FF only, LTV handles pose correction */
  public void drive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    leftController.setSetpoint(
        wheelSpeeds.leftMetersPerSecond, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    rightController.setSetpoint(
        wheelSpeeds.rightMetersPerSecond, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  /**
   * Command to drive the robot
   *
   * @param xSpeed Drive power (throttle). Squared for smoother controls.
   * @param zRotation Rotation in the z axis(around itself). Squared for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return run(
        () -> {
          robotDrive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble(), true);
        });
  }

 public Command closedLoopDriveCommand(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return run(
        () -> {
          var speeds = DifferentialDrive.arcadeDriveIK(
              xSpeed.getAsDouble(), zRotation.getAsDouble(), true);
          double leftMetersPerSec = speeds.left * Constants.DriveConstants.maxSpeed;
          double rightMetersPerSec = speeds.right * Constants.DriveConstants.maxSpeed;

          leftController.setSetpoint(
              leftMetersPerSec, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          rightController.setSetpoint(
              rightMetersPerSec, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        });
} 
  public Command getAutonomousCommand(String string) {
    return new PathPlannerAuto(string);
  }

  public void arcadeDrive(double speed, double rotation) {
    robotDrive.arcadeDrive(speed, rotation);
  }

  public Rotation3d getGyroRotation3d() {
    return gyro.getRotation3d();
  }

  public double getGyroYawRate() {
    return gyro.getRate();
  }

  public void applyVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  /** Call when robot is at start position — records starting encoder values */
public void wheelRadiusCharStart() {
    SmartDashboard.putNumber("WheelChar/StartLeft", leftEncoder.getPosition());
    SmartDashboard.putNumber("WheelChar/StartRight", rightEncoder.getPosition());
    SmartDashboard.putString("WheelChar/Status", "Started — push robot forward, enter distance, then finish");
}

/** Call after pushing robot — reads actual distance from dashboard and prints result */
public void wheelRadiusCharFinish() {
    double startLeft = SmartDashboard.getNumber("WheelChar/StartLeft", 0.0);
    double startRight = SmartDashboard.getNumber("WheelChar/StartRight", 0.0);
    double actualMeters = SmartDashboard.getNumber("WheelChar/ActualDistanceMeters", 0.0);

    double leftDelta = leftEncoder.getPosition() - startLeft;
    double rightDelta = rightEncoder.getPosition() - startRight;
    double encoderAvg = (leftDelta + rightDelta) / 2.0;

    double correctionFactor = actualMeters / encoderAvg;
    double correctedCircumference = Constants.DriveConstants.wheelCircumference * correctionFactor;
    double correctedRadiusMeters = correctedCircumference / (2 * Math.PI);
    double correctedRadiusInches = correctedRadiusMeters / 0.0254;

    SmartDashboard.putNumber("WheelChar/CorrectedRadiusInches", correctedRadiusInches);
    SmartDashboard.putNumber("WheelChar/CorrectedCircumference", correctedCircumference);
    SmartDashboard.putNumber("WheelChar/CorrectionFactor", correctionFactor);
    SmartDashboard.putString("WheelChar/Status", "Done!  wheelCircumference: " + correctedCircumference + "  wheelRadiusInches: " + correctedRadiusInches);
}

  @Override
  public void periodic() {
    // Display the applied output of the left and right side onto the dashboard
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    SmartDashboard.putNumber("LEFT 1 POWER", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("LEFT 2 POWER", leftMotorFollower.getAppliedOutput());
    SmartDashboard.putNumber("RIGHT POWER", rightMotor.getAppliedOutput());
    SmartDashboard.putNumber("RIGHT 2 POWER", rightMotorFollower.getAppliedOutput());
    SmartDashboard.putNumber("Left Speed m s", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Speed m s", rightEncoder.getVelocity());
    SmartDashboard.putNumber("navx", gyro.getYaw());
    SmartDashboard.putNumber("navx2", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Left Position m", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position m", rightEncoder.getPosition());
    m_field.setRobotPose(getPose());
    // This method will be called once per scheduler run
  }
}