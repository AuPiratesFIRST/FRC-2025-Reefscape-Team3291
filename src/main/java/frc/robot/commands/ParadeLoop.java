// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.commands.ElevatorCMDs.GoToFloor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ParadeLoop extends SequentialCommandGroup {
  /** Creates a new ParadeLoop. */
  private final ElevatorSubsystem elevatorSubsystem = RobotContainer.elevatorSubsystem;
  private final IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;
  private BooleanSupplier filler = () -> false; // Placeholder for unused BooleanSuppliers
  private final GoToFloor groundFloor = new GoToFloor(elevatorSubsystem, intakePivotSubsystem, filler, filler, filler, filler, 0);
  private final GoToFloor topFloor = new GoToFloor(elevatorSubsystem, intakePivotSubsystem, filler, filler, filler, filler, 3);
  

  public ParadeLoop() {
        addCommands(new SequentialCommandGroup(
          groundFloor.until(() -> elevatorSubsystem.ifAtFloor(Constants.Elevator.groundFloor)), // Move up
          new WaitCommand(1.0),                          // Wait 1 second
          topFloor.until(() -> elevatorSubsystem.ifAtFloor(Constants.Elevator.fourthFloor)),// Move down
          new WaitCommand(1.0)                         // Wait 1 second
            // Stop motor briefly
  ).repeatedly());
  System.out.println("ParadeLoop initialized with ground and top floor commands.");
    
    

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
