// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 2;
  }

  public static class DriveConstants {
    public static final int leftMotor = 12; // CAN ID
    public static final int leftFollower = 10; // CAN ID
    public static final int rightMotor = 17; // CAN ID
    public static final int rightFollower = 18; // CAN ID

    public static final double trackWidthMeters = 0.5505; // meters
    public static final double gearRatio = 10.714285;
    public static final double maxSpeed = 4.227576; // meters per second
    public static final double maxAngularVelocity = 2.0 * maxSpeed / trackWidthMeters; // rad/s

    public static final double wheelCircumference = Units.inchesToMeters(6.0 * Math.PI);
    public static final double velocityConversionFactor =
        (1.0 / gearRatio) * (wheelCircumference) / 60.0;
    public static final double positionConversionFactor = (1.0 / gearRatio) * (wheelCircumference);
  }

  public static class LimelightConstants {
    public static final String limelightName = "limelight";
    public static final int aprilTagPipeline = 0;
    public static final double hubX = 4.625; // meters, blue alliance origin
    public static final double hubY = 4.0349; // meters, blue alliance origin
    public static final double searchSpeed = 0.35;
    public static final double aimKP = 0.02;
    public static final double aimTolerance = 2.0; // degrees
  }

  public static class FeederConstants {
    public static final int feederMotor = 15; // CAN ID
  }

  public static class IntakeConstants {
    public static final int intakeMotor = 13; // CAN ID
    public static final double intakeSpeed = 0.6;
  }

  public static class ShooterConstants {
    public static final int shooterMotorLeft = 14; // CAN ID
    public static final int shooterMotorRight = 16; // CAN ID
    public static final double shooterSpeed = 0.8;
    public static final double kS = 0.0; // Volts, static gain
    public static final double kV = 0.00211; // Volts per RPM, velocity gain
    public static final double kA = 0.0; // Volts per (meter per second squared), acceleration gain
    public static final double kP = 0.00005; // Proportional gain
    public static final double kI = 0.0; // Integral gain
    public static final double kD = 0.0; // Derivative gain
    public static final double kF = 0.0; // Feedforward gain
  }

  public static class ClimberConstants {
    public static final int climberMotor = 11; // CAN ID
  }
}
