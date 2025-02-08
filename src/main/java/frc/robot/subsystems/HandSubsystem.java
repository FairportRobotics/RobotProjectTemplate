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
  // public SparkClosedLoopController m_controller = handMotor.getClosedLoopController();
  private boolean hazPiece;
  /** Creates a new ExampleSubsystem. */
  public HandSubsystem() {
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(limitSwitch.)


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
  
}
