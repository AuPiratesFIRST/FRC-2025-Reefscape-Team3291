package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class VisionSubsystem {

    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-front");

    public void updateLimelightData() {
        // Get Limelight data using LimelightHelpers
        boolean hasTarget = LimelightHelpers.getTargetCount("limelight") > 0;
        double xOffset = LimelightHelpers.getTX("limelight");
        double yOffset = LimelightHelpers.getTY("limelight");
        //double distance = LimelightHelpers.getDistance("limelight");
        double tagID = LimelightHelpers.getFiducialID("limelight");

        // Send data to NetworkTables
        limelight.getEntry("tv").setDouble(hasTarget ? 1.0 : 0.0);
        limelight.getEntry("tx").setDouble(xOffset);
        limelight.getEntry("ty").setDouble(yOffset);
        //limelight.getEntry("tz").setDouble(distance);
        limelight.getEntry("tid").setDouble(tagID);
    }

    public double limelightRangeProportional(String limelight)
    {    
      double kP = 0.1;//change later

      double targetingForwardSpeed = LimelightHelpers.getTY(limelight) * kP;
      targetingForwardSpeed *= Constants.Swerve.maxSpeed; // 3.0 m/s max speed
      targetingForwardSpeed *= -1.0;

      return targetingForwardSpeed;
    }

    public double limelight_aim_proportional(String limelight)
    {    
      double kP = .035;//change later
  
      double targetingAngularVelocity = LimelightHelpers.getTX(limelight) * kP;
  
      // convert to radians per second for our drive method
      targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;
  
      //invert since tx is positive when the target is to the right of the crosshair
      targetingAngularVelocity *= -1.0;
  
      return targetingAngularVelocity;
    }

    public double getLimelightTagID(String limelight) {
        return LimelightHelpers.getFiducialID(limelight);
    }

}
