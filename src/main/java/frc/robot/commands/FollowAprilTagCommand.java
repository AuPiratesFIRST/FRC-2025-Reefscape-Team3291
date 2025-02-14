package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class FollowAprilTagCommand extends Command {
    private final SwerveSubsystem swerve;
    private Pose2d targetPose;
    private Command driveCommand;
    private boolean hasFinished;

    public FollowAprilTagCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        targetPose = swerve.getAprilTagPose();
        driveCommand = null; // Reset any previous command
        System.out.println("FollowAprilTagCommand Initialized");
        hasFinished = false;
    }

    @Override
    public void execute() {
    Pose2d newPose = swerve.getAprilTagPose();

    if (newPose != null && !newPose.equals(targetPose)) {
        targetPose = newPose;
        System.out.println("New Target Pose: " + targetPose);
        if (driveCommand == null || driveCommand.isFinished()) {
            swerve.drive(new Translation2d(targetPose.getX(), targetPose.getY()), targetPose.getRotation().getRotations(), true);
            //driveCommand = swerve.driveToPose(targetPose);
            if (driveCommand != null) {
                System.out.println("Scheduling new drive command...");
                driveCommand.schedule();
            } else {
                System.out.println("driveToPose() returned null!");
            }
        }
    }

    hasFinished = true;
}

    @Override
    public boolean isFinished() {
        return false; // Keeps running until interrupted
    }

}
