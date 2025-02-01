package frc.robot.commands;

import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class TestElevatorUpCommand extends ElevatorGoToLevelCommand
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
        super(elevatorSubsystem, getLevel(elevatorSubsystem.getLevel()));
    }

    /**
     * Get the next level of the elevator.
     * @param currentLevel the current level of the elevator.
     * @return the next level of the elevator.
     */
    private static ElevatorLevels getLevel(ElevatorLevels currentLevel) {
        if(validToMoveUp(currentLevel))
            return ElevatorLevels.values()[currentLevel.ordinal() + 1];
        return ElevatorLevels.NONE;
    }

    /**
     * Check if the elevator is not at the top level.
     * @param currentLevel The current level of the elevator.
     * @return true if the elevator is not at the top level.
     */
    private static boolean validToMoveUp(ElevatorLevels currentLevel) {
        return currentLevel != ElevatorLevels.MAX;
    }
}
