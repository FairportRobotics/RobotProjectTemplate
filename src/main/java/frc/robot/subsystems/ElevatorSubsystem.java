package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.commands.ElevatorGoToLevelCommand;

public class ElevatorSubsystem extends SubsystemBase {
    @FunctionalInterface
    public interface PositionGetter {
        double getPos(TalonFX motor);
    }

    // The home positions of the elevator motors, initially we don't know the home
    // positions of the elevator.
    private double leftHomePos = Double.MAX_VALUE;
    private double rightHomePos = Double.MAX_VALUE;

    // The motors of the elevator.
    private TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorMotors.LEFT_ID),
            elevatorRightMotor = new TalonFX(Constants.ElevatorMotors.RIGHT_ID);

    // The bottom limit switch of the elevator.
    private DigitalInput bottomlimitSwitch;

    // The level that the elevator should go to.
    private volatile ElevatorLevels goToLevel = Constants.ElevatorLevels.HOME;

    // Used to extract the most up-to-date position of the inputted motor.
    private PositionGetter getterPos = (motor) -> {
        return motor.getPosition().getValueAsDouble();
    };

    // Logic variables for the periodic method.
    private boolean isChangingLevel = false, isCalibrated = false;
    private int skipCycles = 0;

    public ElevatorSubsystem() {
        // toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(Constants.ElevatorLimitSwitches.BOTTOM_ID);

        TalonFXConfiguration elevatorMotor1Config = getDefaultTalonFXConfiguration();
        elevatorMotor1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);
        elevatorLeftMotor.getPosition().setUpdateFrequency(50);
        elevatorLeftMotor.optimizeBusUtilization();
        // elevatorMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration elevatorMotor2Config = getDefaultTalonFXConfiguration();
        elevatorMotor2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorRightMotor.getConfigurator().apply(elevatorMotor2Config);
        elevatorRightMotor.getPosition().setUpdateFrequency(50);
        elevatorRightMotor.optimizeBusUtilization();
        // elevatorMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    }

    /**
     * Constructs a TalonFXConfig with a default PID.
     */
    private static TalonFXConfiguration getDefaultTalonFXConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 1.7;
        config.Slot0.kI = 1.5;
        config.Slot0.kD = 0.3;
        return config;
    }

    /**
     * Gets the level of the elevator.
     */
    public ElevatorLevels getGoToLevel() {
        return goToLevel;
    }

    /**
     * Sets the level of the elevator.
     * 
     * @param newLevel is the level to set the elevator to.
     *                 If the new level is null, the elevator is already at the
     *                 level, or the elevator is not initialized, this method does
     *                 nothing.
     */
    public void setLevel(ElevatorLevels newLevel) {
        if (newLevel == null || goToLevel.equals(newLevel) || elevatorNotInitialized())
            return;
        goToLevel = newLevel;
        // The elevator must change its position now.
        isChangingLevel = true;
    }

    /**
     * (This the initializer of the elevator subsystem when the robot is enabled)
     * Moves the elevator down while the leftHomePos or rightHomePos are not set.
     * When the bottom limit switch is pressed, leftHomePos and rightHomePos are
     * initialized and the motors are stopped.
     */
    @Override
    public void periodic() {
        System.out.println(goToLevel.toString());
        if (isChangingLevel) {
            // Moves the elevator to the level
            moveElevator();
            // The elevator is already moving, default checks can now continue.
            isChangingLevel = false;
            // The home position will recalibrate the next time the elevator reaches home
            // position.
            isCalibrated = false;
            /*
             * If the home position is not the level that the elevator should be going to,
             * then 5 cycles of recalibrating the home positions (~100 ms) are skipped to
             * ensure enough time for the elevator to deactivate the bottom limit switch
             * and not trigger a recalibration stop.
             */
            if (!ElevatorLevels.HOME.equals(goToLevel))
                skipCycles = 5;
        } else
            defaultPeriodic();
    }

    /**
     * The default periodic method for the elevator.
     * This includes recalibration of the home positions and moving the elevator
     * down if the home positions are not initialized.
     */
    private void defaultPeriodic() {
        /**
         * If the bottom limit switch is pressed, the home positions are not already
         * calibrated, and this check is not intentially skipped, recalibrate the home
         * position.
         */
        if (!bottomlimitSwitch.get() && !isCalibrated && skipCycles == 0) {
            elevatorLeftMotor.set(0.0);
            elevatorRightMotor.set(0.0);

            setHomePositions(getterPos.getPos(elevatorLeftMotor), getterPos.getPos(elevatorRightMotor));

            elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
            elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);

            // The home positions are now calibrated
            isCalibrated = true;
            return;
        }

        // Decrements skip cycles if it is used for the check above.
        if (skipCycles > 0)
            skipCycles--;

        /**
         * If robot home positions are not initialized, keep moving down.
         */
        if (elevatorNotInitialized()) {
            elevatorLeftMotor.set(-0.1);
            elevatorRightMotor.set(-0.1);
            return;
        }
    }

    /**
     * Checks if the elevator is not initialized.
     * 
     * @return true if the elevator is not initialized, false otherwise.
     */
    public boolean elevatorNotInitialized() {
        return leftHomePos == Double.MAX_VALUE || rightHomePos == Double.MAX_VALUE;
    }

    @Deprecated
    /**
     * Gets the left motor of the elevator.
     * 
     * @return the left motor of the elevator.
     */
    public TalonFX getLeftMotor() {
        return elevatorLeftMotor;
    }

    @Deprecated
    /**
     * Gets the right motor of the elevator.
     * 
     * @return the right motor of the elevator.
     */
    public TalonFX getRightMotor() {
        return elevatorRightMotor;
    }

    @Deprecated
    /**
     * Sets the speed of the elevator motors.
     * 
     * @param speed is the speed of the motors to be set to.
     */
    public void set(double speed) {
        elevatorLeftMotor.set(speed);
        elevatorRightMotor.set(speed);
    }

    /**
     * Gets the bottom limit switch of the elevator as a boolean.
     * 
     * @return true if the bottom limit switch is pressed, false otherwise.
     */
    public boolean getBottomLimitSwitchAsBoolean() {
        return bottomlimitSwitch.get();
    }

    @Deprecated
    /**
     * Gets the home position of the left motor.
     * 
     * @return the the home position of the left motor.
     */
    public double getLeftHomePos() {
        return leftHomePos;
    }

    @Deprecated
    /**
     * Gets the home position of the right motor.
     * 
     * @return the the home position of the right motor.
     */
    public double getRightHomePos() {
        return rightHomePos;
    }

    @Deprecated
    /**
     * Sets the home position of the left motor.
     * 
     * @param leftHomePos is the home position of the left motor.
     */
    public void setLeftHomePos(double leftHomePos) {
        this.leftHomePos = leftHomePos;
    }

    @Deprecated
    /**
     * Sets the home position of the right motor.
     * 
     * @param rightHomePos is the home position of the right motor.
     */
    public void setRightHomePos(double rightHomePos) {
        this.rightHomePos = rightHomePos;
    }

    /**
     * Sets the home positions of the elevator motors.
     * 
     * @param left  is the home position of the left motor.
     * @param right is the home position of the right motor.
     */
    private void setHomePositions(double left, double right) {
        leftHomePos = left;
        rightHomePos = right;
    }

    /**
     * Moves the elevator to the level specified by goToLevel.
     */
    private void moveElevator() {
        setMotorNeutralMode(NeutralModeValue.Coast);
        setMotorPositions(ElevatorGoToLevelCommand.ENCODER_GETTER.get(goToLevel));
    }

    /**
     * Stops the elevator motors and sets the motors to brake.
     */
    public void stopMotors() {
        elevatorLeftMotor.stopMotor();
        elevatorRightMotor.stopMotor();
        setMotorNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Sets the neutral mode of both elevator motors.
     * 
     * @param modeValue is the neutral mode of the motors.
     */
    private void setMotorNeutralMode(NeutralModeValue modeValue) {
        elevatorLeftMotor.setNeutralMode(modeValue);
        elevatorRightMotor.setNeutralMode(modeValue);
    }

    /**
     * Sets the positions of both elevator motors.
     * 
     * @param position is the position to set the motors to.
     */
    private void setMotorPositions(double position) {
        elevatorLeftMotor.setControl(new PositionVoltage(leftHomePos + position));
        elevatorRightMotor.setControl(new PositionVoltage(rightHomePos + position));
    }
}