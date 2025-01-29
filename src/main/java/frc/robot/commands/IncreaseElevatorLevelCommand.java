package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class IncreaseElevatorLevelCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public IncreaseElevatorLevelCommand(ElevatorSubsystem elevatorSubsystem)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(new Subsystem[] {elevatorSubsystem});
    }

    @Override
    public void execute()
    {
        elevatorSubsystem.increaseLevel();
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
