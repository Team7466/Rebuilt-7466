package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubSystem extends SubsystemBase {
  private final SparkMax IntakeMotor;
  private SparkMaxConfig motorConfig;
    private SparkMaxConfig stopconfig;

  private DigitalInput beamBreak;
  private AnalogInput  infraRed;
  Debouncer debounce ;
    
     public IntakeSubSystem() {
    IntakeMotor = new SparkMax(IntakeConstants.intakeMotor, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    applyConfigs();
    setConfigs();


  }
  /** Set parameters for the SPARK. */
  private void setConfigs() {
    motorConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.15)
        .voltageCompensation(12.0);

    motorConfig
        .signals
        .appliedOutputPeriodMs(10)
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(20);
  }


  private void stopconfig(){
stopconfig
   .idleMode(IdleMode.kBrake);


  }


    private void applyConfigs() {
    IntakeMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void Intakesetspeed(double speed){

    IntakeMotor.set(speed);

  }



  public void Intakestop(){

    IntakeMotor.set(0);
    stopconfig();

  }


}
