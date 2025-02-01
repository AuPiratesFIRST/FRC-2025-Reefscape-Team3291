package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RunMotorSub extends SubsystemBase {

    private final SparkMax motor;

    public RunMotorSub() {
        motor = new SparkMax(22, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void runMotor(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.set(0);
    }
}
