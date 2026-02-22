// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  FeederSubsystem feederSubsystem;
  ShooterSubsystem shooterSubsystem;
  boolean isFinished;
  double RPM;

  /** Creates a new AutoShoot. */
  public AutoShoot(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, double RPM) {
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.RPM = RPM;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(feederSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooterSubsystem.shooterSet(RPM);

    if (shooterSubsystem.isShooterAtSetpoint()) {
      feederSubsystem.feederShoot();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
