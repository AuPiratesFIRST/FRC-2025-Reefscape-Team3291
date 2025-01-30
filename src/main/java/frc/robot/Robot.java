package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.VisionSubsystem;

public class Robot extends TimedRobot {

    private final VisionSubsystem visionSubsystem = new VisionSubsystem();

    @Override
    public void robotPeriodic() {
        // Update Limelight data in NetworkTables
        visionSubsystem.updateLimelightData();
    }
}
