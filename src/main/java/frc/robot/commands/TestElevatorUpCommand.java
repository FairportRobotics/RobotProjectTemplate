package frc.robot.commands;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class TestElevatorUpCommand extends Command
{
    /**
     * Creates a new ElevatorUpCommand.
     * Decorates itself with a new ElevatorGoToLevelCommand if moving the elevator is valid.
     * The execution of the ElevatorGoToLevelCommand would be scheduled after the execution of this command's end method
     * (and hopefully effectively replacing the ElevatorUpCommand with the ElevatorGoToLevelCommand to the intended level in
     * CommandScheduler).
     * @param elevatorSubsystem The elevator subsystem used by this command.
     */
    public TestElevatorUpCommand(ElevatorSubsystem elevatorSubsystem) {
        addRequirements(elevatorSubsystem);
        if(validToMoveUp(elevatorSubsystem.getLevel()))
        {
            BooleanConsumer consumer = command -> {
                CommandScheduler.getInstance().schedule(new ElevatorGoToLevelCommand(elevatorSubsystem, getLevel(elevatorSubsystem.getLevel())));
            };
            this.finallyDo(consumer);
        }
    }

    /**
     * Get the next level of the elevator.
     * @param elevatorLevels the current level of the elevator.
     * @return the next level of the elevator.
     */
    private ElevatorLevels getLevel(ElevatorLevels elevatorLevels) {
        return ElevatorLevels.values()[elevatorLevels.ordinal() + 1];
    }

    /**
     * Check if the elevator is not at the top level.
     * @param currentLevel The current level of the elevator.
     * @return true if the elevator is not at the top level.
     */
    private boolean validToMoveUp(ElevatorLevels currentLevel) {
        return currentLevel != ElevatorLevels.FOUR;
    }

    /**
     * This command is finished upon initialization.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
