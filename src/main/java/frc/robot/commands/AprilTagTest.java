// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagTest extends Command {
  /** Creates a new AprilTagTest. */

  private final VisionSubsystem visionSubsystem;
  private double visionTX;
  private double visionTY;

  public AprilTagTest(VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    //this.swerveSubsystem = swerveSubsystem;
    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(visionSubsystem.isThereATarget() == true){

      visionSubsystem.getApriltagId();

      double out = visionSubsystem.getLimelightSpeed() * visionSubsystem.proportionalAiming();

      visionSubsystem.setMotor(out);

      SmartDashboard.putBoolean("HIIIII", true);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double txTarget = MathUtil.applyDeadband(visionSubsystem.getTXSwerve(), Constants.Vision.XDeadband);
    double tyTarget = MathUtil.applyDeadband(visionSubsystem.getTYSwerve(), Constants.Vision.XDeadband);

    if (txTarget == Constants.Vision.XOffset && tyTarget == Constants.Vision.yTargetValue) {
      return true;
    }
    return false;
  }
}
