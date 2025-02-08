package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToAprilTagCommandSimple extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final LimelightHelpers limelightHelpers;
    private final VisionSubsystem visionSubsystem;
    private Pose2d targetPose;
    private double targetDistance;
    public String frontCamera = "limelight-front";
    public String backCamera = "limelight-back";

    public DriveToAprilTagCommandSimple(SwerveSubsystem swerveSubsystem, LimelightHelpers limelightHelpers, VisionSubsystem visionSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightHelpers = limelightHelpers;
        this.visionSubsystem = visionSubsystem;

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
            swerveSubsystem.driveCommandSimple(0.0, visionSubsystem.limelightRangeProportional(frontCamera), visionSubsystem.limelight_aim_proportional(frontCamera));
            return;
        }
    }

    @Override
    public boolean isFinished() {
    double txTarget = MathUtil.applyDeadband(LimelightHelpers.getTX(frontCamera), Constants.Vision.XDeadband);
    double tyTarget = MathUtil.applyDeadband(LimelightHelpers.getTY(frontCamera), Constants.Vision.XDeadband);

    if (txTarget == Constants.Vision.XOffset && tyTarget == Constants.Vision.yTargetValue) {
      return true;
    }
    return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally stop the robot when the command ends (either successfully or interrupted)
        //swerveSubsystem.driveToPose(swerveSubsystem.getPose());
    }
}