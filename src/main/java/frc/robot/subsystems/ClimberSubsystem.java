package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberMotor;
  private SparkMaxConfig motorConfig;
  private final RelativeEncoder climbEncoder;

  private final ProfiledPIDController m_controller = new ProfiledPIDController(
    ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD,
    new TrapezoidProfile.Constraints(ClimberConstants.maxVel, ClimberConstants.maxAccel)
);

  public ClimberSubsystem() {
    climberMotor = new SparkMax(ClimberConstants.climberMotor, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    setConfigs();
    applyConfigs();
    climbEncoder = climberMotor.getAlternateEncoder();

    m_controller.setTolerance(ClimberConstants.posTolerance);
  }

  /** Set parameters for the SPARK. */
  private void setConfigs() {
    motorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.30)
        .voltageCompensation(12.0);
    motorConfig.alternateEncoder
    .setSparkMaxDataPortConfig()
    .countsPerRevolution(8192);
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

  /** Return the encoder position for external controllers/commands. */
  public double getPosition() {
    return climbEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber", climbEncoder.getPosition());
    SmartDashboard.putNumber("Climber Speed", climbEncoder.getVelocity());
    SmartDashboard.putNumber("Climber Motor Output", climberMotor.getAppliedOutput());
  }

  public Command goToPosition(double target) {
    return this.runOnce(() -> m_controller.reset(this.getPosition())) // Önce profili sıfırla
        .andThen(this.run(() -> {
            double output = m_controller.calculate(this.getPosition(), target);
            this.climberSetSpeed(MathUtil.clamp(output + ClimberConstants.gravityFeedforward, -1.0, 1.0));
        }))
        .until(m_controller::atGoal)
        .finallyDo(this::climberStop);
}
}
