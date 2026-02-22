// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

public class LimelightSubsystem extends SubsystemBase {

  private static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 21, 24, 25, 26, 27);
  private static final Set<Integer> RED_HUB_TAGS = Set.of(2,5, 8, 9, 10, 11);

  private final Limelight limelight;
  private final LimelightPoseEstimator poseEstimator;

  public LimelightSubsystem() {
    limelight = new Limelight(LimelightConstants.limelightName);
    poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
  }

  /**
   * Call from DriveSubsystem's periodic or here — feeds gyro orientation to MegaTag2 every loop so
   * pose estimates are accurate.
   */
  public void updateRobotOrientation(Rotation3d rotation, double yawRateDegPerSec) {
    limelight
        .getSettings()
        .withRobotOrientation(
            new Orientation3d(
                rotation,
                new AngularVelocity3d(
                    DegreesPerSecond.of(0.0),
                    DegreesPerSecond.of(0.0),
                    DegreesPerSecond.of(yawRateDegPerSec))));
  }

  /**
   * Returns true if at least one hub AprilTag belonging to our alliance is currently visible with
   * ambiguity below the threshold.
   */
  public boolean hasValidHubTag() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return false;

    Set<Integer> ourTags =
        alliance.get() == DriverStation.Alliance.Blue ? BLUE_HUB_TAGS : RED_HUB_TAGS;

    for (RawFiducial tag : limelight.getData().getRawFiducials()) {
      if (ourTags.contains(tag.id) && tag.ambiguity < LimelightConstants.maxAmbiguity) {
        return true;
      }
    }
    return false;
  }

  /**
   * Uses MegaTag2 pose to compute the bearing (degrees) the robot must face to point at the hub
   * center. Returns empty if pose is unavailable, ambiguity is too high, or pose is outside the
   * field boundary.
   */
  public OptionalDouble getHubTargetBearingDegrees() {
    Optional<PoseEstimate> estimate = poseEstimator.getAlliancePoseEstimate();

    if (estimate.isEmpty() || !estimate.get().hasData) return OptionalDouble.empty();
    if (estimate.get().getMaxTagAmbiguity() > LimelightConstants.maxAmbiguity)
      return OptionalDouble.empty();

    Pose2d robotPose = estimate.get().pose.toPose2d();

    // Sanity check — field is ~16.54 m x 8.07 m
    if (robotPose.getX() < 0
        || robotPose.getX() > 16.54
        || robotPose.getY() < 0
        || robotPose.getY() > 8.07) {
      return OptionalDouble.empty();
    }

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    Translation2d hubCenter =
        alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red
            ? LimelightConstants.RED_HUB_CENTER
            : LimelightConstants.BLUE_HUB_CENTER;

    Translation2d toHub = hubCenter.minus(robotPose.getTranslation());
    double bearing = Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));
    // Shooter is at the back, so rotate target bearing 180° and wrap to [-180, 180]
    return OptionalDouble.of(MathUtil.inputModulus(bearing + 180.0, -180.0, 180.0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limelight Tag", hasValidHubTag());
    // Orientation is fed by the command via updateRobotOrientation(),
    // which is called every execute() loop while AlignToHubCommand is running.
  }
}
