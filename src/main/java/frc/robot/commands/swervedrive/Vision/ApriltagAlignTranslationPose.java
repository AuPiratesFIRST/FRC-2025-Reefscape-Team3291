// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.Vision;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApriltagAlignTranslationPose extends Command {
  /** Creates a new ApriltagAlign. */

  private final SwerveSubsystem swerve;
  private DoubleSupplier stickSupplier;
  public PhotonCamera camera = new PhotonCamera("FrontCamera");
  
    public ApriltagAlignTranslationPose(SwerveSubsystem swerve, DoubleSupplier stickSupplier) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.swerve = swerve;
      this.stickSupplier = stickSupplier;
      
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

    SmartDashboard.putBoolean("Seeing targets?", hasTargets);

    Translation2d translationModify = new Translation2d(0.0, 0.1524);

    if (hasTargets == true) {

          Translation2d translation = swerve.getTranslationToTarget();

          double stick = stickSupplier.getAsDouble();

          stick = MathUtil.applyDeadband(stick, Constants.vision.stickDeadband);

          if (stick > 0){//might flip
            translation = translation.plus(translationModify);
          } else if (stick < 0) {
            translation = translation.minus(translationModify);
          } else {

          }

          Rotation2d targetYaw = swerve.getRotationToTarget();

          swerve.drive(translation, (-targetYaw.getRadians() * 11.5), false);
        }

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
