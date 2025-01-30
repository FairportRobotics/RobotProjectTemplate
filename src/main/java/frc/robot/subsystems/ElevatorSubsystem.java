package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    public static final Constants.Levels[] levels = Constants.Levels.values();
    private static Constants.Levels currentLevel = Constants.Levels.HOME;

    public ElevatorSubsystem()
    {
        //Initializing electronics
    }
    
    /**
     * Sets the elevator at the set level.
     * @param level is a level the elevator should be set at.
     */
    public void setElevatorLevel(Constants.Levels level) {
        //Figure out if the elevator should go up or down.
        //Move elevator based upon previous condition until the corresponding limit switch is triggered.
        //Update the current level.
    }

    public Constants.Levels getCurrentLevel()
    {
        return currentLevel;
    }
}