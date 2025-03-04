package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import com.ctre.phoenix6.controls.PositionVoltage;

import java.util.Objects;

import org.fairportrobotics.frc.posty.TestableSubsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.commands.ElevatorGoToLevelCommand;
import frc.robot.commands.ElevatorGoToLevelCommand.EncoderGetter;

public class ElevatorSubsystem extends TestableSubsystem {

    // The home positions of the elevator motors, initially we don't know the home
    // positions of the elevator.
    private double leftHomePos = Double.MAX_VALUE, rightHomePos = Double.MAX_VALUE;

    // The motors of the elevator.
    private final TalonFX ELEVATOR_LEFT_MOTOR = applyDefaultSettings(new TalonFX(Constants.ElevatorMotors.LEFT_ID),
            false),
            ELEVATOR_RIGHT_MOTOR = applyDefaultSettings(new TalonFX(Constants.ElevatorMotors.RIGHT_ID), true);

    // Stores the position PID that does the motor control.
    private final PositionVoltage LEFT_POS_VOLTAGE = new PositionVoltage(0).withSlot(0),
            RIGHT_POS_VOLTAGE = new PositionVoltage(0).withSlot(0);

    // Helpers whe it comes to caching outcomes of the suppliers.
    private final DigitalInput BOTTOM_LIMIT_SWITCH = new DigitalInput(Constants.DIOValues.ELEVATORLIMIT);
    private final StatusSignal<Angle> LEFT_POS = ELEVATOR_LEFT_MOTOR.getPosition(),
            RIGHT_POS = ELEVATOR_RIGHT_MOTOR.getPosition();

    // The level that the elevator should go to.
    private ElevatorLevels goToLevel = Constants.ElevatorLevels.HOME;

    // Stores the encoder getter for the elevator for faster access.
    private EncoderGetter encoderGetter = ElevatorGoToLevelCommand.ENCODER_GETTER;

    // Logic variables for the periodic method.
    private boolean elevatorNeedsToStartMoving = false, isBraked = false, goToLevelIsHome = true, isInitialized = false;
    private int skipCycles = 0;

    // The arm subsystem that needs to be communicated with as to not break the arm.
    private ArmSubsystem armSubsystem;

    /**
     * Constructs an ElevatorSubsystem.
     * 
     * @throws NullPointerException if armSubsystem is null.
     */
    public ElevatorSubsystem(ArmSubsystem armSubsystem) {
        super("ElevatorSubsystem");

        registerPOSTTest("Elevator left motor is connected", () -> {
            return ELEVATOR_LEFT_MOTOR.isConnected();
        });

        registerPOSTTest("Elevator left motor is connected", () -> {
            return ELEVATOR_RIGHT_MOTOR.isConnected();
        });

        registerBITTest("Elevator bottomLimitSwitch works", () -> {
            setLevel(ElevatorLevels.HOME);

            waitForCondition(() -> BOTTOM_LIMIT_SWITCH.get(), 20);

            return true;
        });

        Objects.requireNonNull(armSubsystem, "ArmSubsystem is needed to ensure the robot doesn't break.");

        this.armSubsystem = armSubsystem;
    }

    /**
     * Applies default settings to the motor.
     * 
     * @param motor                    is the motor to apply the settings to.
     * @param counterClockwisePositive is true if the motor is counter clockwise
     *                                 positive, false for clockwise positive.
     * @return the motor with the default settings applied.
     */
    private TalonFX applyDefaultSettings(TalonFX motor, boolean counterClockwisePositive) {
        setMotorStatus(NeutralModeValue.Brake);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = .3;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        if (counterClockwisePositive)
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        else
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);
        motor.getPosition().setUpdateFrequency(0); // Prevents the signal from being disabled and allows for manual
                                                   // refreshing.
        motor.optimizeBusUtilization();
        return motor;
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
        if (newLevel == null || goToLevel.equals(newLevel) || !isInitialized() || disableManualControl(newLevel))
            return;
        actuallySetLevel(newLevel);
    }

    /**
     * Sets the level of the elevator.
     * 
     * @param newLevel is the level to set the elevator to.
     */
    private void actuallySetLevel(ElevatorLevels newLevel) {
        goToLevel = newLevel;
        elevatorNeedsToStartMoving = true;
        goToLevelIsHome = ElevatorLevels.HOME.equals(goToLevel);
    }

    /**
     * Disables manual control of the elevator due to checks with other subsystems.
     * 
     * @return true if manual control should be disabled, false otherwise.
     */
    private boolean disableManualControl(ElevatorLevels newLevel) {
        return armCheck(newLevel);
    }

    /**
     * Checks if current level of the elevator is in a bad position for the arm.
     * 
     * @return true if the elevator is in a bad position for the arm, false
     *         otherwise.
     */
    private boolean armCheck() {
        return isArmDown() && goToLevel.ordinal() < ElevatorLevels.CORAL.ordinal();
    }

    /**
     * Checks if the checkLevel is a bad position for the arm.
     * 
     * @param checkLevel is the level to check if it is a bad position for the arm.
     * @return true if the checkLevel is a bad position for the arm, false
     *         otherwise.
     */
    private boolean armCheck(ElevatorLevels checkLevel) {
        Objects.requireNonNull(checkLevel, "checkLevel should not be null");
        return isArmDown() && checkLevel.ordinal() < ElevatorLevels.CORAL.ordinal();
    }

    /**
     * Checks if the arm is down.
     * 
     * @return true if the arm is down, false otherwise.
     */
    private boolean isArmDown() {
        return Constants.ArmConstants.ArmPositions.DOWN.equals(armSubsystem.getArmPos());
    }

    /**
     * This method is run constantly and is responsible for moving the elevator to
     * the proper position while also ensuring that the elevator is moving towards
     * or at the goToLevel.
     */
    @Override
    public void periodic() {
        if (armCheck()) {
            actuallySetLevel(ElevatorLevels.CORAL);
            return;
        }
        // If the elevator is not moving and the elevator does not need to move,
        // don't
        // continue additional checks.
        if (isBraked && !elevatorNeedsToStartMoving)
            return;

        // Refresh the positions of the motors for this periodic cycle.
        refreshPositions();

        // If the elevator needs to start moving and the goToLevel is not HOME
        // (continuousChecks handles moving to home), start moving the elevator to
        // the
        // goToLevel position.
        if (elevatorNeedsToStartMoving && !goToLevelIsHome) {
            startMovingElevator();
            elevatorNeedsToStartMoving = false;
        }
        if (!isBraked)
            continuousChecks();
    }

    /**
     * This method runs everytime periodic is run as long as the fuction was not
     * exited. This method is responsible for initializing/recalibrating home
     * positions, continuously updating the speed of the motors when moving down to
     * home and stopping the motors when the elevator is close enough to the
     * goToLevel. This method assumes that the elevator is currently moving.
     * 
     * Checks are prioritized and are responsible as following:
     * 1. Stopping motors and updating home positions when the bottom limit switch
     * is pressed. (Overrides other checks)
     * 2. Constantly updating the speed of the motors when moving down to home.
     * (Includes initialization)
     * 3. Stopping motors when the elevator is close enough to the goToLevel.
     */
    private void continuousChecks() {
        /**
         * If he bottom limit switch is pressed, and either the elevator is not
         * initialized or this check not intentionally skipped, then stop the elevator
         * and update the home positions.
         */
        if (skipCycles == 0 && BOTTOM_LIMIT_SWITCH.get()) {
            setMotorStatus(NeutralModeValue.Brake);
            setHomePositions(LEFT_POS.getValueAsDouble(), RIGHT_POS.getValueAsDouble());
            return;
        } else if (skipCycles > 0) // If the check was intentionally skipped, decrement the skip cycles and
                                   // continue other checks.
            skipCycles--;

        /**
         * If the goToLevel is HOME and the bottom limit switch is not pressed,
         * move down towards home. (continuously updates the speed of the motors)
         */
        if (goToLevelIsHome && !BOTTOM_LIMIT_SWITCH.get()) {
            moveDown();
            return;
        }

        /**
         * If the goToLevel is not HOME, and the difference between the current elevator
         * position and the goToLevel position is less than 0.1, stop the motors.
         */
        if (!goToLevelIsHome
                && Math.abs(LEFT_POS.getValueAsDouble() - (encoderGetter.get(goToLevel) + leftHomePos)) <= 0.1) {
            setMotorStatus(NeutralModeValue.Brake);
            return;
        }
    }

    /**
     * Refreshes the positions of the motors in the status signals.
     */
    private void refreshPositions() {
        LEFT_POS.refresh();
        RIGHT_POS.refresh();
    }

    /**
     * Checks if the elevator is initialized
     * 
     * @return true if the elevator is initialized, false if the elevator is
     *         initialized.
     */
    private boolean isInitialized() {
        if (isInitialized) // Shortcut check to prevent unnecessary calculations if the elevator is already
                           // initialized.
            return true;
        // If both home positions are not their default values, the elevator is
        // initialized.
        return isInitialized = !(leftHomePos == Double.MAX_VALUE || rightHomePos == Double.MAX_VALUE);
    }

    /**
     * Moves the elevator down to the home position.
     */
    public void moveDown() {
        if (!goToLevelIsHome) {
            goToLevel = ElevatorLevels.HOME;
            goToLevelIsHome = true;
        }
        double speed;
        if (isInitialized())
            speed = -0.01; // Math.min(-0.175 * (((double) LEFT_POS.getValueAsDouble() + leftHomePos)
        // / encoderGetter.get(ElevatorLevels.values()[ElevatorLevels.values().length -
        // 1])), -0.035);
        else
            speed = -0.01;
        if (isBraked)
            setMotorStatus(NeutralModeValue.Coast);
        ELEVATOR_LEFT_MOTOR.set(speed);
        ELEVATOR_RIGHT_MOTOR.set(speed);
    }

    /**
     * Gets the bottom limit switch of the elevator as a boolean.
     * 
     * @return true if the bottom limit switch is pressed, false otherwise.
     */
    public boolean getBottomLimitSwitchAsBoolean() {
        return BOTTOM_LIMIT_SWITCH.get();
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
     * Moves the elevator to the level specified by goToLevel and sets skipCycles
     * accordingly. (Should only be used for its utilization with position voltage
     * by setting a specific position and not to anywhere where a limit switch is
     * utilized)
     */
    private void startMovingElevator() {
        setMotorStatus(NeutralModeValue.Coast);
        setMotorPositions(encoderGetter.get(goToLevel));
        /*
         * If the home position is not the level that the elevator should be going to,
         * then 5 cycles of recalibrating the home positions (~100 ms) are skipped to
         * ensure enough time for the elevator to deactivate the bottom limit switch
         * and not trigger a recalibration stop in case the elevator is at the bottom
         * currently.
         * skipCycles is set to 0 (should be checking immediately) if the elevator is
         * going to the home position.
         */
        if (!goToLevelIsHome)
            skipCycles = 5;
        else
            skipCycles = 0;
    }

    /**
     * Sets the neutral mode of both elevator motors.
     * Updates the isBraked variable accordingly and stops the motors automatically
     * if set to brake.
     * 
     * @param modeValue is the neutral mode of the motors.
     * @throws NullPointerException if modeValue is null.
     */
    public void setMotorStatus(NeutralModeValue modeValue) {
        Objects.requireNonNull(modeValue);
        isBraked = NeutralModeValue.Brake.equals(modeValue);
        if (isBraked) {
            ELEVATOR_LEFT_MOTOR.stopMotor();
            ELEVATOR_RIGHT_MOTOR.stopMotor();
        }
        ELEVATOR_LEFT_MOTOR.setNeutralMode(modeValue);
        ELEVATOR_RIGHT_MOTOR.setNeutralMode(modeValue);
    }

    /**
     * Sets the positions of both elevator motors.
     * 
     * @param position is the position to set the motors to.
     */
    private void setMotorPositions(double position) {
        ELEVATOR_LEFT_MOTOR.setControl(LEFT_POS_VOLTAGE.withPosition(leftHomePos + position));
        ELEVATOR_RIGHT_MOTOR.setControl(RIGHT_POS_VOLTAGE.withPosition(rightHomePos + position));
    }

    /**
     * Check for if the elevator is done moving.
     * 
     * @return true if the elevator is not moving anymore, false otherwise.
     */
    public boolean isFinishedMoving() {
        return isBraked;
    }
}
