// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.Vision;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApriltagAlignSimple extends Command {
  /** Creates a new ApriltagAlign. */

  private final SwerveSubsystem swerve;
  private final PhotonCamera camera = new PhotonCamera("FrontCamera");

  public ApriltagAlignSimple(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    PhotonPipelineResult result = camera.getLatestResult();

    boolean hasTargets = result.hasTargets();
    double targetYaw = 0;
    double targetDistance = 0;

    if (hasTargets == true) {

      PhotonTrackedTarget target = result.getBestTarget();
          targetYaw = target.getYaw();
          if (target.getFiducialId() == 6 || target.getFiducialId() == 7 || target.getFiducialId() == 8 || target.getFiducialId() == 9 ||target.getFiducialId() == 10 ||target.getFiducialId() == 11 || target.getFiducialId() == 17 || target.getFiducialId() == 18 || target.getFiducialId() == 19 || target.getFiducialId() == 20 || target.getFiducialId() == 21 || target.getFiducialId() == 22){
            targetDistance = PhotonUtils.calculateDistanceToTargetMeters(0.43, 0.387, Units.degreesToRadians(10), Units.degreesToRadians(target.getPitch()));
          } else if (target.getFiducialId() == 12 || target.getFiducialId() == 2){
            targetDistance = PhotonUtils.calculateDistanceToTargetMeters(0.43, 1.56845, Units.degreesToRadians(10), Units.degreesToRadians(target.getPitch()));
          }
        }

        double turn = (0 - (-targetYaw)) * 1 * 11.5;
        double x = (0.102 - targetDistance) * 1 * 4.4196;
        double y = 0.0;
        Translation2d forward = new Translation2d(x, y);

        swerve.drive(forward,turn, false);

      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    camera.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: Add end condition for pathplanner 
        double targetDistance = MathUtil.applyDeadband(swerve.distanceToTarget(), Constants.vision.deadband);
    double targetYaw = MathUtil.applyDeadband(swerve.getTargetYaw(), Constants.vision.deadband);

    if (targetDistance == 0 && targetYaw == 0) {
      return true;
    }

    return false;
  }
}
