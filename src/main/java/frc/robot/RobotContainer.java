// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.FeederSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_DriveSubsystem = new DriveSubsystem();

  FuelSubsystem m_FuelSubsystem = new FuelSubsystem();
  FeederSubsystem m_FeederSubsystem = new FeederSubsystem();

  private double speed = 0.9;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static final CommandPS5Controller driverPS =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  public static final CommandPS5Controller operatorPS =
      new CommandPS5Controller(OperatorConstants.kOperatorControllerPort);

  public final CommandXboxController operatorXbox =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_DriveSubsystem.setDefaultCommand(
        new DriveCommand(
            m_DriveSubsystem, () -> driverPS.getLeftY() * speed, () -> 0.8 * -driverPS.getRightX()));
    m_FuelSubsystem.setDefaultCommand(
        m_FuelSubsystem.run(() -> m_FuelSubsystem.fuelStop()));
    m_FeederSubsystem.setDefaultCommand(
        m_FeederSubsystem.run(() -> m_FeederSubsystem.feederSetSpeed(0.0)));
    // Setup auto chooser

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    driverPS.R1().whileTrue(Commands.run(() -> speed = 0.6));

    driverPS.R1().onFalse(Commands.run(() -> speed = 0.9));
    driverPS.L1().whileTrue(m_FuelSubsystem.run(() -> m_FuelSubsystem.fuelSetSpeed(0.83)));
    driverPS.L1().whileTrue(m_FeederSubsystem.run(() -> m_FeederSubsystem.feederSetSpeed(1.0)));
    
    
    driverPS.circle().whileTrue(m_FuelSubsystem.run(() -> m_FuelSubsystem.fuelSetSpeed(0.83)));
    driverPS.circle().whileTrue(m_FeederSubsystem.run(() -> m_FeederSubsystem.feederSetSpeed(1.0)));// motor

    driverPS.L2().whileTrue(m_FeederSubsystem.run(() -> m_FeederSubsystem.feederSetSpeed(-1.0)));
    driverPS.L2().whileTrue(m_FuelSubsystem.run(() -> m_FuelSubsystem.fuelSetSpeed(0.83)));




  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
