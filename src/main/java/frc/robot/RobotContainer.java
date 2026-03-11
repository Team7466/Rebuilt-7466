// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToHubCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();

  private double speed = 1.0;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // public static final CommandXboxController driverXbox =
  //    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static final CommandPS5Controller driverPS =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  public static final CommandPS5Controller operatorPS =
      new CommandPS5Controller(OperatorConstants.kOperatorControllerPort);

  public final CommandXboxController operatorXbox =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_DriveSubsystem.setDefaultCommand(
        m_DriveSubsystem.driveCommand(
            () -> speed * -driverPS.getLeftY(), () -> 0.7 * -driverPS.getRightX()));

    m_IntakeSubsystem.setDefaultCommand(
        m_IntakeSubsystem.run(() -> m_IntakeSubsystem.intakeStop()));

    m_FeederSubsystem.setDefaultCommand(
        m_FeederSubsystem.run(() -> m_FeederSubsystem.feederStop()));

    m_ClimberSubsystem.setDefaultCommand(
        m_ClimberSubsystem.run(() -> m_ClimberSubsystem.climberStop()));

    m_shooterSubsystem.setDefaultCommand(
        m_shooterSubsystem.run(() -> m_shooterSubsystem.shooterStop()));

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

    /*
     *
     * DRIVER CONTROLS:
     R1: yarım hız modu
     * 
     */

    driverPS.R1().whileTrue(Commands.run(() -> speed = 0.5)); // hiz ayar butonu

    /*
     * 
     * OPERATOR CONTROLS:
      L2: intake ve feeder birlikte\
      R2 : shooter ve feeder birlikte 
      X : shooter ve feeder birlikte (manuel pid yok)
      DAIRE : sadece intake
      KARE : sadece feeder
      UCGEN : limelight ile hub'a hizalanma
      DPAD UP : climber up
      DPAD DOWN : climber down
     * 
     */

    driverPS.L2().onTrue(new IntakeCommand(m_IntakeSubsystem, m_FeederSubsystem)); //  intake ve feeder

    driverPS.cross().whileTrue(new ShootCommand(m_FeederSubsystem, m_shooterSubsystem, 2750.0)); // shooter ve feeder 

    driverPS.R2().whileTrue(m_IntakeSubsystem.run(() -> m_IntakeSubsystem.intake())); // sadece intake
    driverPS.square().whileTrue(m_FeederSubsystem.run(() -> m_FeederSubsystem.feederIntake())); // sadece feeder

    driverPS.circle().whileTrue(new ManualShootCommand(m_FeederSubsystem, m_shooterSubsystem)); // shooter ve feeder (manuel pid yok)


    driverPS
        .povUp()
        .whileTrue(m_ClimberSubsystem.run(() -> m_ClimberSubsystem.climberSetSpeed(0.5))); // tırmanış
    driverPS
        .povDown()
        .whileTrue(m_ClimberSubsystem.run(() -> m_ClimberSubsystem.climberSetSpeed(-0.5))); // climber geri



    driverPS.triangle().whileTrue(new AlignToHubCommand(m_DriveSubsystem, m_LimelightSubsystem, true));

    /*
     * 
      butonlar bırakıldığında yapılacak işlemler
     */

    driverPS.L1().onTrue(m_IntakeSubsystem.run(() -> m_IntakeSubsystem.intakeSetSpeed(-0.7)).withTimeout(1.0));
    driverPS.L1().onTrue(m_FeederSubsystem.run(() -> m_FeederSubsystem.feederIntake()).withTimeout(1.0));
    driverPS.R1().onFalse(Commands.runOnce(() -> speed = 1.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
