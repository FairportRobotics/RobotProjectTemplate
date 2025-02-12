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
import frc.robot.commands.ElevatorGoToLevelCommand.EncoderGetter;

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

    private EncoderGetter encoderGetter = ElevatorGoToLevelCommand.ENCODER_GETTER;

    // Used to extract the most up-to-date position of the inputted motor.
    private PositionGetter getterPos = (motor) -> {
        return motor.getPosition().getValueAsDouble();
    };

    // Logic variables for the periodic method.
    private boolean isChangingLevel = false, isBraked = false;
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
        config.Slot0.kP = .9;
        config.Slot0.kI = .3;
        config.Slot0.kD = 0;
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
        System.out.println(isBraked);
        if (isChangingLevel) {
            // Moves the elevator to the level
            moveElevator();
            // The elevator is already moving, default checks can now continue.
            isChangingLevel = false;
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
         * If the elevator is not braking (was moving), the home positions are
         * not already calibrated, bottom limit switch is
         * pressed, and this check is not intentially skipped, recalibrate the home
         * position.
         */
        if (!isBraked && !bottomlimitSwitch.get() && skipCycles == 0) {
            stopMotors();

            setHomePositions(getterPos.getPos(elevatorLeftMotor), getterPos.getPos(elevatorRightMotor));

            return;
        }

        // Decrements skip cycles if it is used for the check above.
        if (skipCycles > 0)
            skipCycles--;

        /**
         * If the elevator is not braking (was moving) and the difference
         * between the current elevator position and the goToLevel position is less than
         * 1, stop the motors.
         */
        if (!isBraked && !ElevatorLevels.HOME.equals(goToLevel)
                && Math.abs(getterPos.getPos(elevatorLeftMotor) + leftHomePos - encoderGetter.get(goToLevel)) <= 1) {
            stopMotors();
            return;
        }

        /**
         * If the elevator is not initialized, move the elevator down.
         */
        if (elevatorNotInitialized()) {
            moveDown();
            return;
        }
    }

    public void moveDown() {
        setMotorNeutralMode(NeutralModeValue.Coast);
        isBraked = false;
        elevatorLeftMotor.set(-0.075);
        elevatorRightMotor.set(-0.075);
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
        isBraked = false;
        if (!ElevatorLevels.HOME.equals(goToLevel))
            setMotorPositions(ElevatorGoToLevelCommand.ENCODER_GETTER.get(goToLevel));
        else
            moveDown();
    }

    /**
     * Stops the elevator motors and sets the motors to brake.
     */
    public void stopMotors() {
        elevatorLeftMotor.stopMotor();
        elevatorRightMotor.stopMotor();
        setMotorNeutralMode(NeutralModeValue.Brake);
        isBraked = true;
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