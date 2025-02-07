// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
 
/** An example command that uses an example subsystem. */
public class intakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final HandSubsystem = m_subsystem;
  /**
   * Creates a new IntakeCommand.
   *  IntakeCommand spins the motor in the hand inwards allowing the hand to intake coral
   * 
   * @param subsystem The ArmSubsystem. You know... the thing... that does... the thing...
   */
  public IntakeCommand(HandSubsystem subsystem) {
    m_subsystem = subsystem;
    pos = m_subsystem.getPos();
    if (pos.ordinal() < ArmPositions.DOWN.ordinal()) {
      pos = ArmPositions.values()[pos.ordinal() - 1];
    }


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_subsystem.m_controller.setReference(pos.getValue(), ControlType.kPosition);
    m_subsystem.setPos(pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
