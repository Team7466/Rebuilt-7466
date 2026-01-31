// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FuelSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
   FuelSubsystem fuelSubsystem;
   FeederSubsystem feederSubsystem;

  /** Creates a new ShootCommand. */
  public ShootCommand(FuelSubsystem fuelSubsystem, FeederSubsystem feederSubsystem) {
    this.fuelSubsystem = fuelSubsystem;
    this.feederSubsystem = feederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      fuelSubsystem.run(() -> fuelSubsystem.fuelSetSpeed(0.5))
      .withTimeout(1.0)
      .andThen(feederSubsystem.run(() -> feederSubsystem.feederShoot()))
      .alongWith(fuelSubsystem.run(() -> fuelSubsystem.fuelShoot()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
