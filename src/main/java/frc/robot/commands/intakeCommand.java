// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
 

public class IntakeCommand extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    HandSubsystem _HandSubsystem;
    double intakeSpeed;

    VelocityVoltage intakeRequest;

    public IntakeCommand(HandSubsystem intakeSubsystem, double intakeSpeed){
        _HandSubsystem = intakeSubsystem;
        this.intakeSpeed = intakeSpeed;

        addRequirements(_HandSubsystem);

        intakeRequest = new VelocityVoltage(0).withSlot(0);
    }

    @Override
    public void initialize() {
        _HandSubsystem.intakeLeftMotor.setControl(intakeRequest.withVelocity(intakeSpeed));
//        _IntakeSubsystem.intakeRightMotor.set(ControlMode.PercentOutput, intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        _HandSubsystem.intakeLeftMotor.stopMotor();
    }
}