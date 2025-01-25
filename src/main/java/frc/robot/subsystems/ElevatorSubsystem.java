// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  public SparkMax leftmotor;
  public SparkMax rightmotor;
  //public ColorChanger colorChanger;
  
  public ElevatorSubsystem() {
    
    this.leftmotor = new SparkMax(2, MotorType.kBrushless);//
    this.rightmotor = new SparkMax(1, MotorType.kBrushless);//
    //this.colorChanger = new ColorChanger();
   
  }
   
  @Override
  public void periodic() {
  }

public void setElevatorIndividual(double positiveSpeed, double negativeSpeed) {
 leftmotor.set(-positiveSpeed); 
 rightmotor.set(-negativeSpeed);
}
public void setElevator(double positiveSpeed, double negativeSpeed) {
 leftmotor.set(positiveSpeed - negativeSpeed); 
 rightmotor.set(-1 * (positiveSpeed - negativeSpeed));
}

public void stop() {
 leftmotor.set(0);
 rightmotor.set(0);
    // TODO Auto-generated method stub
}
}
