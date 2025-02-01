package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem {

    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public void updateLimelightData() {
        // Get Limelight data using LimelightHelpers
        boolean hasTarget = LimelightHelpers.getTargetCount("limelight") > 0;
        double xOffset = LimelightHelpers.getTX("limelight");
        double yOffset = LimelightHelpers.getTY("limelight");
        double distance = LimelightHelpers.getDistance("");
        double tagID = LimelightHelpers.getTagID("");

        // Send data to NetworkTables
        limelight.getEntry("tv").setDouble(hasTarget ? 1.0 : 0.0);
        limelight.getEntry("tx").setDouble(xOffset);
        limelight.getEntry("ty").setDouble(yOffset);
        limelight.getEntry("tz").setDouble(distance);
        limelight.getEntry("tid").setDouble(tagID);
    }
}
