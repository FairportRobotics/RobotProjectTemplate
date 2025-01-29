package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class DecreaseElevatorLevelCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public DecreaseElevatorLevelCommand(ElevatorSubsystem elevatorSubsystem)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(new Subsystem[] {elevatorSubsystem});
    }

    @Override
    public void execute()
    {
        elevatorSubsystem.decreaseLevel();
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
