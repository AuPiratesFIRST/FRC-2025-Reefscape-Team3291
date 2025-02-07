package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RunMotorSub2;

public class RunMotorWithAprilTag2 extends Command {

    private final RunMotorSub2 runMotorSub2;
    private final DoubleSupplier speed;

    public RunMotorWithAprilTag2(RunMotorSub2 runMotorSub2, DoubleSupplier speed) {
        this.runMotorSub2 = runMotorSub2;
        this.speed = speed;
        addRequirements(runMotorSub2);
    }

    @Override
    public void execute() {
        double Speed = speed.getAsDouble();
        runMotorSub2.runMotor(Speed); // Now it actually controls the motor
    }

    @Override
    public void end(boolean interrupted) {
        runMotorSub2.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs indefinitely unless interrupted
    }
}
