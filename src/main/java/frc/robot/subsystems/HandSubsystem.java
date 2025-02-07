// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.DigestInputStream;
import java.util.logging.Handler;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.ArmPositions;


public class HandSubsystem extends SubsystemBase {

  
  public SparkMax handMotor = new SparkMax(10, MotorType.kBrushless);
  public DigitalInput limitSwitch = new DigitalInput(1);
  public SparkClosedLoopController m_controller = handMotor.getClosedLoopController();
  /** Creates a new ExampleSubsystem. */
  public HandSubsystem() {
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(pos == ArmPositions.NONE){
            
            this.handMotor.set(0.1);
            }
      }

  }
  /**
   * Get the value of the curent set position for the arm.
   *
   * @return an ArmPositions object that is curently set in the Subsystem. So you can know what position the arm is curently set to. It's kinda usefull.
   */
  public ArmPositions getPos(){
    return pos;
  }

  /**
   * Set the value of the arm position.
   *
   *@param newPos New ArmPositions object to go to. This is important for keeping track of where the arm is. Maybe.
   */
  public void setPos(ArmPositions newPos){
    
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
  
}
