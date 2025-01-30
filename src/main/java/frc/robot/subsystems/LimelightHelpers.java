//LimelightHelpers v1.2.1 (March 1, 2023)

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelpers {

    private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    // Check if the Limelight has detected a target
    public static boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0.0) == 1.0;
    }

    // Get the X offset of the target (horizontal angle)
    public static double getXOffset() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    // Get the Y offset of the target (vertical angle)
    public static double getYOffset() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    // Get the estimated distance to the target
    public static double getDistance() {
        return limelight.getEntry("tz").getDouble(0.0);
    }

    // Get the tag ID of the detected AprilTag
    public static double getTagID() {
        return limelight.getEntry("tid").getDouble(-1);
    }
}
