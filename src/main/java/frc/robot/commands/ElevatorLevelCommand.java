package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorLevelCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private boolean increaseLevelIfTrue;
    private Constants.ElevatorLevels setLevel;
    private Constants.ElevatorLevels[] levels = Constants.ElevatorLevels.values();
    private static boolean sucessfullyChangedLevel = false;

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
    public ElevatorLevelCommand(ElevatorSubsystem elevatorSubsystem, Constants.ElevatorLevels setLevel)
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
            sucessfullyChangedLevel = changeElevatorLevel();
    }

    /**
     * Sets the level of the elevator, this should be the only way HOME could be set on the elevator.
     */
    private void setElevatorLevel()
    {
        elevatorSubsystem.setElevatorLevel(setLevel);
    }

    /**
     * Changes the level of the elevator relative to the target level of the elevator by one level.
     * Ensures that the change will not be out of the range of intended levels (Avoids going past FOUR or going into HOME).
     * (A seprate button would be configured to set the elevator to HOME).
     * @return true if the level was changed, false if the level was not changed.
     */
    private boolean changeElevatorLevel()
    {
        int currentLevelIndex = elevatorSubsystem.getTargetLevel().ordinal(); //Gets the enum index of the target level.
        if(increaseLevelIfTrue)
            if(currentLevelIndex + 1 < levels.length) //If increasing is not out of bounds.
                return elevatorSubsystem.setElevatorLevel(levels[currentLevelIndex + 1]);
            else
                return false;
        else
            if(currentLevelIndex - 1 > 0) //If decreasing is not going into HOME or out of bounds.
                return elevatorSubsystem.setElevatorLevel(levels[elevatorSubsystem.getTargetLevel().ordinal() - 1]);
            else
                return false;
    }

    @Override
    public boolean isFinished()
    {
        return true; /* This command should only be run once everytime this command is constructed
                        (This means using a button's onTrue instead of whileTrue in RobotContainer's configureBindings method). */
    }

    /**
     * Gets if the level was successfully changed using increasing or decreasing last time.
     * @return true if the level was successfully changed, false if the level was not successfully changed.
     */
    public static boolean getSucessfullyChangedLevel()
    {
        return sucessfullyChangedLevel;
    }
}