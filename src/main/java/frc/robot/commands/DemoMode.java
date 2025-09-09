package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ElevatorCMDs.GoToTop;
import frc.robot.commands.ElevatorCMDs.GoToGround;
import frc.robot.commands.ElevatorCMDs.GoToFloor;
import frc.robot.commands.IntakePivotCMDs.PivotToGround;
import frc.robot.commands.IntakePivotCMDs.PivotToStow;
import frc.robot.commands.IntakeMotorCMDs.IntakeCMD;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;
import frc.robot.commands.ColorChangingCMD;
import frc.robot.subsystems.ColorChanger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import java.util.function.BooleanSupplier;


public class DemoMode {
  
  public static Command build(
    ElevatorSubsystem elevator,
    IntakePivotSubsystem intakePivot,
    IntakeMotorSubsystem intakeMotor,
    ColorChanger colorChanger,
    BooleanSupplier stopCondition
  ) {

    elevator.setDemoModeConstraints(); // Set slower constraints for demo mode

    return Commands.sequence(
      
      Commands.waitSeconds(3),

        // Initial Stow Position
     new PivotToGround(intakePivot).withTimeout(2),
        Commands.waitSeconds(2),

        // Step 1: Go to Top
        new GoToTop(elevator).withTimeout(30),
        Commands.waitSeconds(1),

        // Step 2: Go to Ground
        new GoToGround(elevator).withTimeout(3),
        Commands.waitSeconds(1), 

        // Step 3: Go to each floor using GoToFloor (manual triggers replaced with fixed target floors)
        new GoToFloor(elevator, intakePivot, () -> false, () -> false, () -> false, () -> false, 0).withTimeout(3),
        Commands.waitSeconds(1),
        new GoToFloor(elevator, intakePivot, () -> false, () -> false, () -> false, () -> false, 1).withTimeout(3),
        Commands.waitSeconds(1),
        new GoToFloor(elevator, intakePivot, () -> false, () -> false, () -> false, () -> false, 2).withTimeout(3),
        Commands.waitSeconds(1),
        new GoToFloor(elevator, intakePivot, () -> false, () -> false, () -> false, () -> false, 3).withTimeout(3),
        Commands.waitSeconds(1),

        // Step 4: Pivot To Ground
        new PivotToGround(intakePivot).withTimeout(2),
        Commands.waitSeconds(1),

        // Step 5: Pivot To Stow
        new PivotToStow(intakePivot).withTimeout(2),
        Commands.waitSeconds(1),

        // Step 6: Intake
        new IntakeCMD(intakeMotor).withTimeout(2),
        Commands.waitSeconds(1),

        // Step 7: Spit
        new ESpitCMD(intakeMotor).withTimeout(2),
        Commands.waitSeconds(1),

        // Step 8: Color changing (run for a few seconds)
        new ColorChangingCMD(colorChanger).withTimeout(5)
    ).repeatedly().until(stopCondition);
  }
}
