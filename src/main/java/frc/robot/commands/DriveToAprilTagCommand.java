// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.LimelightHelpers;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
// import edu.wpi.first.math.geometry.Pose2d;

// public class DriveToAprilTagCommand extends Command {
//     private final SwerveSubsystem swerveSubsystem;
//     private final LimelightHelpers limelightHelpers;
//     private Pose2d targetPose;

//     public DriveToAprilTagCommand(SwerveSubsystem swerveSubsystem, LimelightHelpers limelightHelpers) {
//         this.swerveSubsystem = swerveSubsystem;
//         this.limelightHelpers = limelightHelpers;
        
//         // Get the Pose2d relative to the AprilTag
//         this.targetPose = limelightHelpers.getBotPose2d("limelight");
        
//         if (targetPose == null) {
//             System.out.println("Pose from Limelight is invalid!");
//             cancel();
//         }

//         // Use the swerve subsystem as a requirement for this command
//         addRequirements(swerveSubsystem);
//     }

//     @Override
//     public void initialize() {
//         // Reset odometry to the vision pose
//         swerveSubsystem.resetOdometry(swerveSubsystem.getPose());
//     }

//     @Override
//     public void execute() {
//         // Drive the swerve system to the target pose
//         swerveSubsystem.driveToPose(targetPose);
//     }

//     @Override
//     public boolean isFinished() {
//         Pose2d currentPose = swerveSubsystem.getPose();
//         return currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.1 // tolerance
//             && Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()) < 0.1; // rotation tolerance
//     }

//     @Override
//     public void end(boolean interrupted) {
//         // Optionally stop the robot when the command ends (either successfully or interrupted)
//         swerveSubsystem.driveToPose(swerveSubsystem.getPose());
//     }
// }