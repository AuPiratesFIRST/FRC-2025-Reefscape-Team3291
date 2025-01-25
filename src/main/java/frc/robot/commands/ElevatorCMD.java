package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorCMD extends Command {
  /** Creates a new MoveIntakeMotorCMD. */
  //ColorChanger colorChanger;
  ElevatorSubsystem elevatorSubsystem;
  DoubleSupplier positiveSupplier;
  DoubleSupplier negativeSupplier;
  public ElevatorCMD(
    ElevatorSubsystem elevatorSubsystem,
    //ColorChanger colorChanger,
    DoubleSupplier positiveSupplier,
    DoubleSupplier negativeSupplier
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    //this.colorChanger = colorChanger;
    addRequirements(elevatorSubsystem);

    this.positiveSupplier = positiveSupplier;
    this.negativeSupplier = negativeSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double positiveSpeed = positiveSupplier.getAsDouble();
    double negativeSpeed = negativeSupplier.getAsDouble();  
}

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
