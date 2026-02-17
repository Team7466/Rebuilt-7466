package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor;
  private SparkMaxConfig motorConfig;
  Debouncer debounce;

  public IntakeSubsystem() {
    intakeMotor = new SparkMax(IntakeConstants.intakeMotor, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    setConfigs();
    applyConfigs();
  }

  /** Set parameters for the SPARK. */
  private void setConfigs() {
    motorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.20)
        .voltageCompensation(12.0);

    motorConfig
        .signals
        .appliedOutputPeriodMs(10)
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(20);
  }

  private void applyConfigs() {
    intakeMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intakeSetSpeed(double speed) {

    intakeMotor.set(speed);
  }

  public void intakeStop() {

    intakeMotor.set(0.0);
  }

  public void intake() {
    intakeMotor.set(0.75);
  }

  @Override
  public void periodic() {}
}
