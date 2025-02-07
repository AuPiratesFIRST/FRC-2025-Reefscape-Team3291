// package frc.robot;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class LimelightAprilTag {
//     private final NetworkTable limelightTable;

//     public LimelightAprilTag() {
//         limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
//     }

//     public boolean hasTarget() {
//         return limelightTable.getEntry("tv").getDouble(0) == 1; // 1 if a target is detected
//     }

//     public int getTagID() {
//         return (int) limelightTable.getEntry("tid").getDouble(-1); // Get the AprilTag ID
//     }

//     public double getXOffset() {
//         return limelightTable.getEntry("tx").getDouble(0.0); // X offset (horizontal)
//     }

//     public double getYOffset() {
//         return limelightTable.getEntry("ty").getDouble(0.0); // Y offset (vertical)
//     }

//     public double getDistance() {
//         return limelightTable.getEntry("tz").getDouble(0.0); // Distance (Z translation)
//     }

//     public double[] getBotPose() {
//         return NetworkTableInstance.getDefault()
//             .getTable("limelight")
//             .getEntry("botpose_wpiblue") // Use "botpose_wpired" for red alliance
//             .getDoubleArray(new double[6]); // X, Y, Z, Roll, Pitch, Yaw
//     }

//     public boolean isTargetValid(int targetID) {
//         int detectedID = getTagID();
//         return hasTarget() && (detectedID == targetID);
//     }
    
    
//     public void updateDashboard() {
//         SmartDashboard.putNumber("AprilTag ID", getTagID());
//         SmartDashboard.putNumber("X Offset", getXOffset());
//         SmartDashboard.putNumber("Y Offset", getYOffset());
//         SmartDashboard.putNumber("Distance", getDistance());
//         SmartDashboard.putBoolean("Has Target", hasTarget());
//     }

    
    
// }
