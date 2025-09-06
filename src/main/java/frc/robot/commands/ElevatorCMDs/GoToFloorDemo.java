// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCMDs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.FloorTarget;
import frc.robot.subsystems.intake.IntakePivotSubsystem;

/**
 * Demo version of GoToFloor that automatically completes after setting the target floor.
 * This command is designed for autonomous sequences where manual input is not needed.
 */
public class GoToFloorDemo extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakePivotSubsystem intakePivotSubsystem;
  private final int targetFloor;
  private final boolean algaeMode;
  private boolean hasSetTarget = false;

  /**
   * Creates a new GoToFloorDemo command.
   * 
   * @param elevatorSubsystem The elevator subsystem
   * @param intakePivotSubsystem The intake pivot subsystem
   * @param targetFloor The floor to go to (0-3)
   * @param algaeMode Whether to use algae mode positioning
   */
  public GoToFloorDemo(ElevatorSubsystem elevatorSubsystem, 
                       IntakePivotSubsystem intakePivotSubsystem, 
                       int targetFloor, 
                       boolean algaeMode) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakePivotSubsystem = intakePivotSubsystem;
    this.targetFloor = targetFloor;
    this.algaeMode = algaeMode;
    
    addRequirements(elevatorSubsystem);
    addRequirements(intakePivotSubsystem);
  }

  @Override
  public void initialize() {
    hasSetTarget = false;
    elevatorSubsystem.algaeMode = algaeMode;
    
    // Set the elevator target based on the floor
    switch (targetFloor) {
      case 0:
        if (algaeMode) {
          elevatorSubsystem.setTarget(FloorTarget.ALGAE_FLOOR);
        } else {
          elevatorSubsystem.setTarget(FloorTarget.GROUND_FLOOR);
        }
        break;
      case 1:
        elevatorSubsystem.setTarget(FloorTarget.SECOND_FLOOR);
        break;
      case 2:
        elevatorSubsystem.setTarget(FloorTarget.THIRD_FLOOR);
        break;
      case 3:
        elevatorSubsystem.setTarget(FloorTarget.FOURTH_FLOOR);
        break;
      default:
        elevatorSubsystem.setTarget(FloorTarget.GROUND_FLOOR);
        break;
    }
    
    // Set the intake pivot target based on the floor and algae mode
    setIntakePivotTarget();
    
    hasSetTarget = true;
    
    SmartDashboard.putNumber("demo current elevator floor", targetFloor);
    SmartDashboard.putBoolean("demo algae mode", algaeMode);
  }

  @Override
  public void execute() {
    // The command just sets the target once and then waits for completion
    // The elevator subsystem handles the actual movement in its periodic() method
  }

  @Override
  public void end(boolean interrupted) {
    // Clean up if needed
  }

  @Override
  public boolean isFinished() {
    // Command finishes immediately after setting the target
    // The timeout in the sequence will handle the actual movement time
    return hasSetTarget;
  }
  
  /**
   * Sets the intake pivot target based on the current floor and algae mode.
   */
  private void setIntakePivotTarget() {
    switch (targetFloor) {
      case 0:
        if (algaeMode) {
          intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.ALGAE;
        } else if (elevatorSubsystem.elevatorEncoder.get() / Constants.Elevator.encoderTicksPerRotation < Constants.Elevator.heightOfHeadBang) {
          intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.STOW;
        }
        break;
      case 1:
        if (algaeMode) {
          intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.GROUND;
        } else {
          intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.MIDLEVELS;
        }
        break;
      case 2:
        if (algaeMode) {
          intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.GROUND;
        } else if (intakePivotSubsystem.midLevelsAngle < Constants.Intake.angleToAvoidHeadBang && 
                   elevatorSubsystem.elevatorEncoder.get() / Constants.Elevator.encoderTicksPerRotation < Constants.Elevator.heightOfHeadBang) {
          intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.TOPLEVEL;
        } else {
          intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.MIDLEVELS;
        }
        break;
      case 3:
        intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.TOPLEVEL;
        break;
    }
  }
}
