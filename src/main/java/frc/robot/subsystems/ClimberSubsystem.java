package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberMotor;
  private SparkMaxConfig motorConfig;
  private final RelativeEncoder climbEncoder;
  private final SparkClosedLoopController climberController;

  public ClimberSubsystem() {
    climberMotor = new SparkMax(ClimberConstants.climberMotor, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    setConfigs();
    applyConfigs();
    climbEncoder = climberMotor.getEncoder();
    climberController = climberMotor.getClosedLoopController();
    resetEncoder();
  }

  /** Set parameters for the SPARK. */
  private void setConfigs() {
    motorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.30)
        .closedLoopRampRate(0.25)
        .voltageCompensation(12.0);

    motorConfig
        .signals
        .appliedOutputPeriodMs(100)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityPeriodMs(100);

    // Position PID — slot 0
    motorConfig.closedLoop
        .pid(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, ClosedLoopSlot.kSlot0)
        .outputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput, ClosedLoopSlot.kSlot0);

    // Software limits to protect the mechanism from over-travel
    motorConfig.softLimit
        .forwardSoftLimit(ClimberConstants.forwardSoftLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(ClimberConstants.reverseSoftLimit)
        .reverseSoftLimitEnabled(true);
  }

  private void applyConfigs() {
    climberMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Command the climber to a specific encoder position (rotations). */
  public void setPosition(double rotations) {
    climberController.setSetpoint(
        rotations,
        SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  public void climberSetSpeed(double speed) {
    climberMotor.set(speed);
  }

  public void climberStop() {
    climberMotor.set(0.0);
  }

  public void climb() {
    climberMotor.set(0.75);
  }

  public double getClimbEncoder() {
    return climbEncoder.getPosition();
  }

  public void resetEncoder() {
    climbEncoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber", getClimbEncoder());
    SmartDashboard.putNumber("Climber Speed", climbEncoder.getVelocity());
    SmartDashboard.putNumber("Climber Motor Output", climberMotor.getAppliedOutput());
  }
}
