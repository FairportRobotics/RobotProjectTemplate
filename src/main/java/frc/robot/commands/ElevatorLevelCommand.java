package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorLevelCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private boolean increaseLevelIfTrue;
    private Constants.Levels setLevel;
    private Constants.Levels[] levels = Constants.Levels.values();

    /**
     * Constructs the ElevatorLevelCommand, either increasing or decreasing the level of the elevator when executed.
     * @param elevatorSubsystem is the elevator subsystem.
     * @param increaseLevelIfTrue is true if the elevator should increase in level, false if the elevator should decrease in level.
     */
    public ElevatorLevelCommand(ElevatorSubsystem elevatorSubsystem, boolean increaseLevelIfTrue)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.increaseLevelIfTrue = increaseLevelIfTrue;
        addRequirements(new Subsystem[] {elevatorSubsystem});
    }

    /**
     * Constructs the ElevatorLevelCommand, setting the elevator level when executed.
     * @param elevatorSubsystem is the elevator subsystem.
     * @param setLevel is the level the elevator should be set to.
     */
    public ElevatorLevelCommand(ElevatorSubsystem elevatorSubsystem, Constants.Levels setLevel)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.setLevel = setLevel;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute()
    {
        if(setLevel != null) /* Checks if the command should be setting the level of the elevator
                                or is either increasing or decreasing the level of the elevator by figuring out if
                                setLevel was set upon construction. */
            setElevatorLevel();
        else
            changeElevatorLevel();
    }

    /**
     * Sets the level of the elevator, this should be the only way HOME could be set on the elevator.
     */
    private void setElevatorLevel()
    {
        elevatorSubsystem.setElevatorLevel(setLevel);
    }

    /**
     * Changes the level of the elevator relative to the current level of the elevator by one level.
     * Ensures that the change will not be out of the range of intended levels (Avoids going past FOUR or going into HOME).
     */
    private void changeElevatorLevel()
    {
        int currentLevelIndex = elevatorSubsystem.getCurrentLevel().ordinal();
        if(increaseLevelIfTrue)
            if(currentLevelIndex + 1 < levels.length) //If increasing is not out of bounds.
                elevatorSubsystem.setElevatorLevel(levels[currentLevelIndex + 1]);
        else
            if(currentLevelIndex - 1 > 0) //If decreasing is not going into HOME or out of bounds.
                elevatorSubsystem.setElevatorLevel(levels[elevatorSubsystem.getCurrentLevel().ordinal() - 1]);
    }

    @Override
    public boolean isFinished()
    {
        return true; /* This command should only be run once everytime this command is constructed
                        (This means using a button's onTrue instead of whileTrue in RobotContainer's configureBindings method). */
    }
}