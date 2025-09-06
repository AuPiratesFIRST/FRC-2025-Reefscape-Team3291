package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ElevatorCMDs.GoToTop;
import frc.robot.commands.ElevatorCMDs.GoToGround;
import frc.robot.commands.ElevatorCMDs.GoToFloorDemo;
import frc.robot.commands.IntakePivotCMDs.PivotToGround;
import frc.robot.commands.IntakePivotCMDs.PivotToStow;
import frc.robot.commands.IntakeMotorCMDs.IntakeCMD;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;
import frc.robot.subsystems.ColorChanger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;

public class DemoMode {
  
  public static Command build(
    ElevatorSubsystem elevator,
    IntakePivotSubsystem intakePivot,
    IntakeMotorSubsystem intakeMotor,
    ColorChanger colorChanger
  ) {
    return Commands.sequence(
        // Step 1: Go to Top
        new GoToTop(elevator).withTimeout(30),
        Commands.waitSeconds(1),

        // Step 2: Go to Ground
        new GoToGround(elevator).withTimeout(3),
        Commands.waitSeconds(1),

        // Step 3: Go to each floor using GoToFloorDemo (auto-completing version for demo mode)
        new GoToFloorDemo(elevator, intakePivot, 0, false).withTimeout(3),
        Commands.waitSeconds(1),
        new GoToFloorDemo(elevator, intakePivot, 1, false).withTimeout(3),
        Commands.waitSeconds(1),
        new GoToFloorDemo(elevator, intakePivot, 2, false).withTimeout(3),
        Commands.waitSeconds(1),
        new GoToFloorDemo(elevator, intakePivot, 3, false).withTimeout(3),
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
    );
  }
}
