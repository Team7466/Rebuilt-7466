// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  FeederSubsystem feederSubsystem;
  ShooterSubsystem shooterSubsystem;
  IntakeSubsystem intakeSubsystem;
  boolean isFinished;
  double RPM;

  /** Creates a new ShootCommand. */
  public ShootCommand(
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      double RPM) {

    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.RPM = RPM;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(intakeSubsystem, feederSubsystem, shooterSubsystem);
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
      intakeSubsystem.intake();
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
