// Copyright (c) 2025 FRC 5449
// http://github.com/frc-team5449
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import frc.robot.lib.util.AllianceFlipUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class FieldConstants {
  public static final AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double fieldWidth = tagLayout.getFieldWidth();
  public static final double fieldLength = tagLayout.getFieldLength();

  public static final Translation2d center =
      new Translation2d(Units.inchesToMeters(176.746), tagLayout.getFieldWidth() / 2.0);

  public static final Translation2d blueReefCenter = new Translation2d(4.378, 3.884);
  public static final Translation2d redReefCenter = new Translation2d(12.943, 3.884);

  public static final Map<Integer, Pose2d> blueCenterFaces =
      new HashMap<>(); // Starting facing the driver station in clockwise order
  public static final Map<Integer, Pose2d> redCenterFaces = new HashMap<>();

  public static final Map<Integer, Integer> blueNumToTag = new HashMap<>();
  public static final Map<Integer, Integer> redNumToTag = new HashMap<>();

  public static final int[] blueTagIds = {17, 18, 19, 20, 21, 22};
  public static final int[] redTagIds = {6, 7, 8, 9, 10, 11};

  public static void initializeField() {
    tagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    blueCenterFaces.put(17, tagLayout.getTagPose(17).get().toPose2d());
    blueCenterFaces.put(18, tagLayout.getTagPose(18).get().toPose2d());
    blueCenterFaces.put(19, tagLayout.getTagPose(19).get().toPose2d());
    blueCenterFaces.put(20, tagLayout.getTagPose(20).get().toPose2d());
    blueCenterFaces.put(21, tagLayout.getTagPose(21).get().toPose2d());
    blueCenterFaces.put(22, tagLayout.getTagPose(22).get().toPose2d());

    redCenterFaces.put(6, tagLayout.getTagPose(6).get().toPose2d());
    redCenterFaces.put(7, tagLayout.getTagPose(7).get().toPose2d());
    redCenterFaces.put(8, tagLayout.getTagPose(8).get().toPose2d());
    redCenterFaces.put(9, tagLayout.getTagPose(9).get().toPose2d());
    redCenterFaces.put(10, tagLayout.getTagPose(10).get().toPose2d());
    redCenterFaces.put(11, tagLayout.getTagPose(11).get().toPose2d());

    redNumToTag.put(0, 7);
    redNumToTag.put(1, 8);
    redNumToTag.put(2, 9);
    redNumToTag.put(3, 10);
    redNumToTag.put(4, 11);
    redNumToTag.put(5, 6);

    blueNumToTag.put(0, 21);
    blueNumToTag.put(1, 20);
    blueNumToTag.put(2, 19);
    blueNumToTag.put(3, 18);
    blueNumToTag.put(4, 17);
    blueNumToTag.put(5, 22);

    Logger.recordOutput("Odometry/BR", blueReefCenter);
    Logger.recordOutput("Odometry/RR", redReefCenter);
  }

  public static final List<Pose2d> targetPoses =
      new ArrayList<Pose2d>(
          DriverStation.getAlliance().get() == Alliance.Blue
              ? blueCenterFaces.values()
              : redCenterFaces.values());

  public static final int[] tagIds =
      DriverStation.getAlliance().get() == Alliance.Blue ? blueTagIds : redTagIds;

  public static final double initialHeading =
      DriverStation.getAlliance().get() == Alliance.Red ? 0 : 180;

  public static final Transform2d leftBranchTargetPoseRelativeToTagL4 =
      new Transform2d(0.56, -0.167, Rotation2d.kZero);
  public static final Transform2d rightBranchTargetPoseRelativeToTagL4 =
      new Transform2d(0.56, 0.167, Rotation2d.kZero);

  public static final Transform2d leftBranchTargetPoseRelativeToTag =
      new Transform2d(0.5, -0.167, Rotation2d.kZero);
  public static final Transform2d rightBranchTargetPoseRelativeToTag =
      new Transform2d(0.5, 0.167, Rotation2d.kZero);

  public static Optional<Pose2d> getReefTagPose(Translation2d botPosition) {
    Translation2d dist =
        botPosition.minus(AllianceFlipUtil.shouldFlip() ? redReefCenter : blueReefCenter);
    if (dist.getNorm() > 4) {
      return Optional.empty();
    }
    int remainder = ((((int) Math.floor((dist.getAngle().getDegrees() + 30) / 60)) % 6) + 6) % 6;
    Integer reefNumber =
        (AllianceFlipUtil.shouldFlip() ? redNumToTag : blueNumToTag).get(remainder);
    return Optional.of(
        (AllianceFlipUtil.shouldFlip() ? redCenterFaces : blueCenterFaces).get(reefNumber));
  }
}
