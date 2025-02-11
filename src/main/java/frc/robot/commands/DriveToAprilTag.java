package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public class DriveToAprilTag extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final LimelightHelpers limelightHelpers;
    private final VisionSubsystem visionSubsystem;
    private Pose2d targetPose;
    public String frontCamera = "limelight-front";
    public String backCamera = "limelight-back";

    public DriveToAprilTag(SwerveSubsystem swerveSubsystem, LimelightHelpers limelightHelpers, VisionSubsystem visionSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightHelpers = limelightHelpers;
        this.visionSubsystem = visionSubsystem;
        
        // Get the Pose2d relative to the AprilTag
        this.targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(backCamera).toPose2d();
        //double targetDistance = targetPose.getTranslation().getDistance(new Translation3d());//may be nothing
        
        if (targetPose == null) {
            System.out.println("Pose from Limelight is invalid!");
            cancel();
        }

        // Use the swerve subsystem as a requirement for this command
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Reset odometry to the vision pose
        swerveSubsystem.resetOdometry(swerveSubsystem.getPose());
    }

    @Override
    public void execute() {
        // Drive the swerve system to the target pose
        if (visionSubsystem.getLimelightTagID(frontCamera) == 1) {
            swerveSubsystem.driveToPose(targetPose);
            return;
        }
        
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = swerveSubsystem.getPose();
        return currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.1 // tolerance
            && Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()) < 0.1; // rotation tolerance
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally stop the robot when the command ends (either successfully or interrupted)
        swerveSubsystem.driveToPose(swerveSubsystem.getPose());
    }
}