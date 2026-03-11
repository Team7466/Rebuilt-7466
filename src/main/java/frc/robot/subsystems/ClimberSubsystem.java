package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberMotor;
  private SparkMaxConfig motorConfig;
  private final RelativeEncoder climbEncoder;

  public ClimberSubsystem() {
    climberMotor = new SparkMax(ClimberConstants.climberMotor, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    setConfigs();
    applyConfigs();
    climbEncoder = climberMotor.getAlternateEncoder();
  }

  /** Set parameters for the SPARK. */
  private void setConfigs() {
    motorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.30)
        .voltageCompensation(12.0);
    motorConfig.alternateEncoder.countsPerRevolution(8192);
    motorConfig
        .signals
        .appliedOutputPeriodMs(100)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityPeriodMs(100);
  }

  private void applyConfigs() {
    climberMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber", climbEncoder.getPosition());
    SmartDashboard.putNumber("Climber Speed", climbEncoder.getVelocity());
    SmartDashboard.putNumber("Climber Motor Output", climberMotor.getAppliedOutput());
  }
}
