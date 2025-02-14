package frc.robot.subsystems;

import com.limelightvision.limelight.frc.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName = "limelight-front"; // Change if using a different name

    public double[] getAprilTagPose() {
        return LimelightHelpers.getBotPose(limelightName);
    }

    public boolean hasTarget() {
        return getAprilTagPose().length >= 6; // Ensure valid pose data
    }
}
