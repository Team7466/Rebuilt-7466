// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
    public static final int leftMotor = 12; // CAN ID — leader, has through bore encoder
    public static final int leftFollower = 10; // CAN ID
    public static final int rightMotor = 18; // CAN ID — leader, has through bore encoder
    public static final int rightFollower = 17; // CAN ID

    public static final double trackWidthMeters = 0.5505; // meters
    public static final double gearRatio = 8.4586;
    public static final double maxSpeed = 4.74; // meters per second
    public static final double maxAngularVelocity = 2.0 * maxSpeed / trackWidthMeters; // rad/s

    public static final double wheelCircumference = Units.inchesToMeters(6.0 * Math.PI);
    // Built-in NEO encoder conversion factors (motor shaft — includes gear ratio)
    // Kept in case we revert to hall sensor
    public static final double velocityConversionFactor =
        (1.0 / gearRatio) * (wheelCircumference) / 60.0;
    public static final double positionConversionFactor = (1.0 / gearRatio) * (wheelCircumference);

    // Feedforward in volts (REV FF takes volts)
    // kP in duty cycle divide ReCalc value by 12
    public static final double kS = 0.0; // volts 
    public static final double kV = 2.15; // V*s/m
    public static final double kP = 2.57 /2 / 12.0; // duty cycle per (m/s) of error
  }

  public static class LimelightConstants {  
    public static final String limelightName = "limelight";
    public static final int aprilTagPipeline = 0;

    // Hub centers in WPILib field coords (meters, blue-origin).
    // From 2026 official field dimension drawings:
    //   Blue hub X = 181.56 in * 0.0254 = 4.612 m from blue alliance wall
    //   Hub Y      = 158.32 in * 0.0254 = 4.021 m (lateral center)
    //   Red hub X  = (651.22 - 181.56) in * 0.0254 = 11.929 m
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.612, 4.021);
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.929, 4.021);

    public static final double maxAmbiguity = 0.2;

    public static final double searchSpeed = 0.35;
    public static final double aimKP = 0.05;
    public static final double aimKI = 0.0;
    public static final double aimKD = 0.000;
    public static final double aimTolerance = 1.5; // degrees
    public static final double aimMaxSpeed = 0.5;
  }

  public static class FeederConstants {
    public static final int feederMotor = 15; // CAN ID
  }

  public static class IntakeConstants {
    public static final int intakeMotor = 13; // CAN ID
  }

  public static class ShooterConstants {
    public static final int shooterMotorLeft = 14; // CAN ID
    public static final int shooterMotorRight = 16; // CAN ID
    public static final double shooterSpeed = 0.8;
    public static final double kS = 0.0; // Volts, static gain
    public static final double kV = 0.00212; // Volts per RPM, velocity gain
    public static final double kA = 0.0; // Volts per (meter per second squared), acceleration gain
    public static final double kP = 0.00045; // Proportional gain
    public static final double kI = 0.0; // Integral gain
    public static final double kD = 0.0; // Derivative gain
    public static final double kF = 0.0; // Feedforward gain
  }

  public static class ClimberConstants {
    public static final int climberMotor = 11; // CAN ID

    // Position PID — tune these on the robot
    public static final double kP = 0.0009 ;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kMinOutput = -0.45;
    public static final double kMaxOutput = 0.65;

    // Soft limits in motor rotations — tune for your mechanism travel
    public static final double forwardSoftLimit = 150.0;  // fully climbed
    public static final double reverseSoftLimit = 0.0;    // fully retracted
  }
}