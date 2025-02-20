// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApriltagAlignSimple1 extends Command {
  /** Creates a new ApriltagAlign. */

  private final SwerveSubsystem swerve;
  private final Vision vision;
  private final int id;
  private final PhotonCamera camera = new PhotonCamera("FrontCamera");
  private PhotonPipelineResult results = camera.getLatestResult();

  public ApriltagAlignSimple1(SwerveSubsystem swerve, Vision vision, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.swerve = swerve;
    this.id = id;
    this.addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PhotonCamera camera = new PhotonCamera("FrontCamera");
    PhotonPipelineResult result = camera.getLatestResult();

    boolean hasTargets = result.hasTargets();
    int apriltagID = id;
    Pose3d targetPose = vision.getTagPose(0);
    Transform3d cameraToRobot = new Transform3d(
            new Translation3d(0.5, 0.2, 0),  // Translation (x, y, z)
            new Rotation3d(0, 0, 0) // Rotation (roll, pitch, yaw)
        );

    if (hasTargets == true) {

      for (var target : result.getTargets()) {

        if (vision.isThereATag(target.getFiducialId()) == true) {

          targetPose = vision.getTagPose(target.getFiducialId());
          Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), targetPose, cameraToRobot);
          double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose.toPose2d(), targetPose.toPose2d());
          Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(distanceToTarget, Rotation2d.fromDegrees(-target.getYaw()));
          Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose.toPose2d(), targetPose.toPose2d());
          swerve.drive(translation, (targetYaw.getRadians() * 11.5), false);
        }

        }

        //swerve.drive(forward,turn, false);

      }

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
