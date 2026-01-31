package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;

public class FuelSubsystem extends SubsystemBase {
  private final SparkMax fuelMotor;
  private SparkMaxConfig motorConfig;
  private SparkMaxConfig stopconfig;

  private DigitalInput beamBreak;
  private AnalogInput infraRed;
  Debouncer debounce;

  public FuelSubsystem() {
    fuelMotor = new SparkMax(FuelConstants.fuelMotor, MotorType.kBrushless);
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
    fuelMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void fuelSetSpeed(double speed) {

    fuelMotor.set(speed);
  }

  public void fuelStop() {

    fuelMotor.set(0.0);
  }

  public void fuelShoot() {
    fuelMotor.set(0.81);
  }

  public void fuelIntake() {
    fuelMotor.set(0.75);
  }
}
