package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.subsystems.VisionSubsystem;

public class Robot extends TimedRobot {

    //private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private RobotContainer m_robotcontainer;

    @Override
    public void robotInit(){
        m_robotcontainer = new RobotContainer();
        
    }

    @Override
    public void disabledInit(){
        
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // Update Limelight data in NetworkTables
        //visionSubsystem.updateLimelightData();
    }
}
