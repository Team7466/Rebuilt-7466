package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.OptionalDouble;

/**
 * Rotates the robot to face the hub center.
 *
 * <p>Scans slowly until a valid hub tag is visible, then PIDs to the bearing. Finishes when the PID
 * is at setpoint.
 *
 */
public class AlignToHubCommand extends Command {

  private final DriveSubsystem drive;
  private final LimelightSubsystem limelightSubsystem;
  private final boolean startTurnLeft;
  private final PIDController rotPID;

  public AlignToHubCommand(
      DriveSubsystem drive, LimelightSubsystem limelightSubsystem, boolean startTurnLeft) {
    this.drive = drive;
    this.limelightSubsystem = limelightSubsystem;
    this.startTurnLeft = startTurnLeft;

    rotPID =
        new PIDController(
            LimelightConstants.aimKP, LimelightConstants.aimKI, LimelightConstants.aimKD);
    rotPID.setTolerance(LimelightConstants.aimTolerance);
    rotPID.enableContinuousInput(-180, 180);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    rotPID.reset();
  }

  @Override
  public void execute() {
    limelightSubsystem.updateRobotOrientation(drive.getGyroRotation3d(), drive.getGyroYawRate());

    if (limelightSubsystem.hasValidHubTag()) {
      OptionalDouble bearing = limelightSubsystem.getHubTargetBearingDegrees();
      if (bearing.isPresent()) {
        double rotSpeed =
            rotPID.calculate(
                drive.getGyroRotation3d().toRotation2d().getDegrees(), bearing.getAsDouble());
        drive.arcadeDrive(
            0,
            MathUtil.clamp(
                rotSpeed, -LimelightConstants.aimMaxSpeed, LimelightConstants.aimMaxSpeed));
        return;
      }
    }

    drive.arcadeDrive(
        0, startTurnLeft ? LimelightConstants.searchSpeed : -LimelightConstants.searchSpeed);
  }

  @Override
  public boolean isFinished() {
    return rotPID.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }
}
