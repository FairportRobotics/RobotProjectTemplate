package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HandSubsystem;
public class Wait extends SequentialCommandGroup {
    public Wait(HandSubsystem handSubsystem) {
        super(
            Commands.deadline(new WaitCommand(5.0), new HandCommand(handSubsystem, -.5))
        );
    }
}

