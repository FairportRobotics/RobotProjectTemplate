package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    public final TalonFX elevatorLeft = new TalonFX(Constants.ElevatorMotors.LEFT);
    public final TalonFX elevatorRight = new TalonFX(Constants.ElevatorMotors.RIGHT);
    public final TalonFX elevatorMiddle = new TalonFX(Constants.ElevatorMotors.MIDDLE); // NOTE: Might not be used. Talk to Jordan.
    public final DigitalInput mainElevatorSwitch = new DigitalInput(Constants.ElevatorLimitSwitches.MAIN);
    public final DigitalInput middleElevatorSwitch = new DigitalInput(Constants.ElevatorLimitSwitches.MIDDLE); // NOTE: Might not be used. Talk to Jordan.

    public double currentPosition = elevatorLeft.get();
    public Constants.ElevatorLevels targetLevel;

    /**
     * Constructor for the ElevatorSubsystem.
     * Initializes the subsystem.
     */
    public ElevatorSubsystem() {
        targetLevel = Constants.ElevatorLevels.HOME; // Setting the target level to HOME.
        // Initializing electronics
    }

    /**
     * Sets the target level of the elevator.
     * @param newLevel is the level the elevator is trying to reach.
     * @return true after the target level was set.
     */
    public boolean setElevatorLevel(Constants.ElevatorLevels newLevel) {
        targetLevel = newLevel;
        return true;
    }

    /**
     * This method is called periodically by CommandScheduler and updates the current position of the elevator.
     */
    @Override
    public void periodic() {
        //Updates the current position of the elevator.
        currentPosition = elevatorLeft.get();
        if (!isAtTargetLevel()) //If not at the right place.
        {
            double targetPosition = Constants.getEncoderValueForLevel(targetLevel);
            //Move the elevator to the right place.
            if (currentPosition > targetPosition)
                setMotors(-0.1);
            else if (currentPosition < targetPosition)
                setMotors(0.1);
        }
        else
            //Else, stop the motors.
            setMotors(0);
    }

    /**
     * Sets the motors' speed.
     * @param number is the speed the motors should be set to.
     */
    private void setMotors(double number) {
        elevatorLeft.set(number);
        elevatorRight.set(number);
        elevatorMiddle.set(number);
    }

    /**
     * Checks if the elevator is at the target level.
     * @return true if the elevator is at the target level, otherwise false.
     */
    private boolean isAtTargetLevel(){
        if(Constants.getLimitSwitchForLevel(targetLevel) != null) //Checks if a limit switch is assigned for the target level.
            return Constants.getLimitSwitchForLevel(targetLevel).get(); //Returns true if the limit switch is pressed.
        return Constants.getEncoderValueForLevel(targetLevel) == currentPosition; //Returns true if the elevator is at the correct position.
    }

    /**
     * Gets the current position of the elevator.
     * @return the current position of the elevator.
     */
    public double getCurrentLevel() {
        return currentPosition;
    }

    /**
     * Gets the target level of the elevator.
     * The target level is the level the elevator is trying to reach.
     * @return the target level of the elevator.
     */
    public Constants.ElevatorLevels getTargetLevel() {
        return targetLevel;
    }
}