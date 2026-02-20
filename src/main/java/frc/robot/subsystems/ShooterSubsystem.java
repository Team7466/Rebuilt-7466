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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooterMotorLeft;
  private final SparkMax shooterMotorRight;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private SparkMaxConfig leftConfig;
  private SparkMaxConfig rightConfig;
  private double setpoint;
  private final SparkClosedLoopController shootController;
  private SimpleMotorFeedforward feedforward;

  public ShooterSubsystem() {
    shooterMotorLeft = new SparkMax(ShooterConstants.shooterMotorLeft, MotorType.kBrushless);
    shooterMotorRight = new SparkMax(ShooterConstants.shooterMotorRight, MotorType.kBrushless);
    leftEncoder = shooterMotorLeft.getEncoder();
    rightEncoder = shooterMotorRight.getEncoder();
    leftConfig = new SparkMaxConfig();
    rightConfig = new SparkMaxConfig();
    setConfigs();
    applyConfigs();
    shootController = shooterMotorLeft.getClosedLoopController();
    setpoint = 3000.0;
    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    feedforward =
        new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
  }

  /** Set parameters for the SPARK. */
  private void setConfigs() {
    leftConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.20)
        .voltageCompensation(12.0);

    leftConfig
        .signals
        .appliedOutputPeriodMs(10)
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(10);

    rightConfig.apply(leftConfig).inverted(true);
  }

  private void applyConfigs() {
    shooterMotorLeft.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotorRight.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void shooterSetSpeed(double speed) {

    shooterMotorLeft.set(speed);
    shooterMotorRight.set(speed);
  }

  public void shooterSet(double RPM) {
    setpoint = RPM;

    shootController.setSetpoint(
        setpoint, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, 2.0);
  }

  public void shooterSetVoltage(double voltage) {
    shooterMotorLeft.setVoltage(voltage);
    shooterMotorRight.setVoltage(voltage);
  }

  public Boolean isShooterAtSetpoint() {
    return Math.abs((leftEncoder.getVelocity() - setpoint)) <= 130;
  }

  public void shooterStop() {
    shooterMotorLeft.set(0.0);
    shooterMotorRight.set(0.0);
  }

  public void shooterWindUp() {
    shooterMotorLeft.set(-0.4);
    shooterMotorRight.set(-0.4);
  }

  public void shoot() {
    shooterMotorLeft.set(-0.55);
    shooterMotorRight.set(-0.55);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter L power", shooterMotorLeft.getAppliedOutput());
    SmartDashboard.putNumber("shooter R power", shooterMotorRight.getAppliedOutput());
    SmartDashboard.putNumber("shooter L RPM ", leftEncoder.getVelocity());
    SmartDashboard.putNumber("shooter R RPM ", rightEncoder.getVelocity());
  }
}
