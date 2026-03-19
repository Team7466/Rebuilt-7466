package frc.robot.commands;
/*
 Bu dosyaya gerek kalmadı gibi yazdım da kullanılmıyor bir yerde dursun şuanlık olmadı sileriz. 
 */
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbToPositionCommand extends Command { // Profiled eklendi
  private final ClimberSubsystem m_climber;
  private final ProfiledPIDController m_controller;
  private final double m_targetPosition;

  public ClimbToPositionCommand(ClimberSubsystem climber, double targetPosition) {
    m_climber = climber;
    m_targetPosition = targetPosition;
    m_controller = new ProfiledPIDController(
        ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD,
        new TrapezoidProfile.Constraints(ClimberConstants.maxVel, ClimberConstants.maxAccel)
    );
    m_controller.setTolerance(ClimberConstants.posTolerance);
    addRequirements(m_climber);
 
  }

  @Override
  public void initialize() {
    m_controller.reset(m_climber.getPosition());
  }

  @Override
  public void execute() {
    double output = m_controller.calculate(m_climber.getPosition(), m_targetPosition);
    output = MathUtil.clamp(output+ClimberConstants.gravityFeedforward,-1.0,1.0);
    m_climber.climberSetSpeed(output);
  }

  @Override
  public boolean isFinished() {
    return m_controller.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.climberStop();
  }


}