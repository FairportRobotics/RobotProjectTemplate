package frc.robot.subsystems;

import java.util.logging.Level;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Levels;

public class ElevatorSubsystem extends SubsystemBase {
    public static final Constants.Levels[] levels = Constants.Levels.values();
    private static Constants.Levels currentLevel = Constants.Levels.HOME;
    public TalonFX elevatorLeft = new TalonFX(0);
    public TalonFX elevatorRight = new TalonFX(0);
    public TalonFX elevatorMiddle = new TalonFX(0); // NOTE: Might not be used. Talk to Jordan.
    public DigitalInput mainElevatorSwitch = new DigitalInput(0);
    public DigitalInput middleElevatorSwitch = new DigitalInput(0); // NOTE: Might not be used. Talk to Jorbin.

    public double CURENTPOS = 0;
    public Constants.Levels level;

    public ElevatorSubsystem() {
        // Initializing electronics

    }

    /**
     * Sets the elevator at the set level.
     * 
     * @param level is a level the elevator should be set at.
     */
    public void setElevatorLevel(Constants.Levels newLevel) {
        // Figure out if the elevator should go up or down.
        // Move elevator based upon previous condition until the corresponding limit
        // switch is triggered.
        // Update the current level.
        level = newLevel;
    }

    @Override
    public void periodic() {
        CURENTPOS = elevatorLeft.get();
        if (level.equals(Levels.HOME) && !mainElevatorSwitch.get()) {
            setMotors(-0.1);
        } else if (level.equals(level.ONE) && CURENTPOS != level.getValue()) {
            if (CURENTPOS > level.getValue())
                setMotors(-0.1);

            if (CURENTPOS < level.getValue())
                setMotors(0.1);

        } else if (level.equals(level.TWO) && CURENTPOS != level.getValue()) {
            if (CURENTPOS > level.getValue())
                setMotors(-0.1);

            if (CURENTPOS < level.getValue())
                setMotors(0.1);

        } else if (level.equals(level.THREE) && CURENTPOS != level.getValue()) {
            if (CURENTPOS > level.getValue())
                setMotors(-0.1);

            if (CURENTPOS < level.getValue())
                setMotors(0.1);

        } else if (level.equals(level.FOUR) && CURENTPOS != level.getValue()) {
            if (CURENTPOS > level.getValue())
                setMotors(-0.1);

            if (CURENTPOS < level.getValue())
                setMotors(0.1);

        } else {
            setMotors(0);
        }

    }

    private void setMotors(double number) {
        elevatorLeft.set(number);
        elevatorRight.set(number);
        elevatorMiddle.set(number);
    }
    private void checkPos(Constants.Levels ){
        if (level.equals(level.FOUR) && CURENTPOS != level.getValue()) {
            if (CURENTPOS > level.getValue())
                setMotors(-0.1);

            if (CURENTPOS < level.getValue())
                setMotors(0.1);
        }
    }

    public Constants.Levels getCurrentLevel() {
        return currentLevel;
    }
}