// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.Vision;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApriltagAlignTranslationPose extends Command {
  /** Creates a new ApriltagAlign. */

  private final SwerveSubsystem swerve;
  private final Vision vision;
  private final int id;
  private final PhotonCamera camera = new PhotonCamera("FrontCamera");
  private PhotonPipelineResult results = camera.getLatestResult(); 

  public ApriltagAlignTranslationPose(SwerveSubsystem swerve, Vision vision, int id) {
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
    double targetYaw = 0;
    double targetDistance = 0;

    if (hasTargets == true) {

      for (var target : result.getTargets()) {
        if (target.getFiducialId() == apriltagID) {
          targetYaw = target.getYaw();
          targetDistance = PhotonUtils.calculateDistanceToTargetMeters(0.43, 0.387, Units.degreesToRadians(10), Units.degreesToRadians(target.getPitch()));
          target.getBestCameraToTarget();
          }
        }

        double turn = (0 - targetYaw) * 1 * 11.5;
        double x = (0.102 - targetDistance) * 1 * 4.4196;
        double y = 0.0;
        Translation2d forward = new Translation2d(x, y);

        swerve.drive(forward,turn, false);

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
