package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RunMotorSub2 extends SubsystemBase {

    private final SparkMax motor;
    private NetworkTable limelightTable;

    public RunMotorSub2() {
        motor = new SparkMax(26, MotorType.kBrushless);
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-back");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void runMotor(double speed) {
    double tagID = limelightTable.getEntry("tid").getDouble(-1);
        motor.setVoltage(tagID == 22 ? speed : 0);
    }

    public void stop() {
        motor.setVoltage(0);
    }
}
