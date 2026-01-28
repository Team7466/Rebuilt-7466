package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {

  private final DriveSubsystem drive;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;

  public DriveCommand(
      DriveSubsystem drive,
      DoubleSupplier xSpeed,
      DoubleSupplier zRotation
  ) {
    this.drive = drive;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;

    addRequirements(drive); //
  }

  @Override
  public void execute() {
    drive.robotDrive.arcadeDrive(
        xSpeed.getAsDouble(),
        zRotation.getAsDouble()
    );
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Default command → sürekli çalışır
  }
}
