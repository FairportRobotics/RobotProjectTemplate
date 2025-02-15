package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.commands.ElevatorGoToLevelCommand;
import frc.robot.commands.ElevatorGoToLevelCommand.EncoderGetter;

public class ElevatorSubsystem extends SubsystemBase {
    interface Helper {
        public void scheduleUpdate();
    }

    abstract class AbstractHelper<T> implements Helper // T representing some type of object.
    {
        // Get value should be initialized on construction of its subclasses
        private boolean scheduleUpdate = false;
        // Stores the value of a subclass.
        protected T value;

        /**
         * Constructs a Helper object with a value and a scheduleRunnable.
         * 
         * @param value contains the most recent value of the supplier.
         */
        public AbstractHelper(T value) {
            this.value = value;
        }

        /**
         * Gets the value of the Helper object.
         * 
         * @return the value of the Helper object.
         */
        public final T get() {
            if (scheduleUpdate) {
                update();
                scheduleUpdate = false;
            }
            return value;
        }

        /**
         * Schedules an update for the cached value the next time get is called.
         */
        public final void scheduleUpdate() {
            scheduleUpdate = true;
        }

        /**
         * Requires the subclass to update the value of the Shell object.
         */
        protected abstract void update();
    }

    /**
     * Stores a limit switch and caches its get Boolean value.
     */
    class SwitchHelper extends AbstractHelper<Boolean> {
        private DigitalInput limitSwitch;

        public SwitchHelper(DigitalInput limitSwitch) {
            super(!limitSwitch.get());
            this.limitSwitch = limitSwitch;
        }

        @Override
        protected void update() {
            value = !limitSwitch.get();
        }
    }

    /**
     * Stores a motor position and caches its Double value.
     */
    class MotorToDoubleHelper extends AbstractHelper<Double> {
        private CoreTalonFX motor;

        public MotorToDoubleHelper(CoreTalonFX motor) {
            super(motor.getPosition().getValueAsDouble());
            this.motor = motor;
        }

        @Override
        protected void update() {
            value = motor.getPosition().getValueAsDouble();
        }
    }

    // The home positions of the elevator motors, initially we don't know the home
    // positions of the elevator.
    private double leftHomePos = Double.MAX_VALUE, rightHomePos = Double.MAX_VALUE;

    // The motors of the elevator.
    private TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorMotors.LEFT_ID),
            elevatorRightMotor = new TalonFX(Constants.ElevatorMotors.RIGHT_ID);

    private final PositionVoltage LEFT_POS_VOLTAGE = new PositionVoltage(0).withSlot(0),
            RIGHT_POS_VOLTAGE = new PositionVoltage(0).withSlot(0);

    // Helpers whe it comes to caching outcomes of the suppliers.
    private SwitchHelper bottomLimitSwitch = new SwitchHelper(
            new DigitalInput(Constants.ElevatorLimitSwitches.BOTTOM_ID));
    private MotorToDoubleHelper leftPos = new MotorToDoubleHelper(elevatorLeftMotor);
    private MotorToDoubleHelper rightPos = new MotorToDoubleHelper(elevatorRightMotor);

    // The level that the elevator should go to.
    private volatile ElevatorLevels goToLevel = Constants.ElevatorLevels.HOME;

    // Stores the encoder getter for the elevator for faster access.
    private EncoderGetter encoderGetter = ElevatorGoToLevelCommand.ENCODER_GETTER;

    // Logic variables for the periodic method.
    private volatile boolean isChangingLevel = true, isBraked = false;
    private volatile int skipCycles = 0;

    // Stores all the registered helpers.
    private Helper[] helpers = { bottomLimitSwitch, leftPos, rightPos };

    public ElevatorSubsystem() {
        // toplimitSwitch = new DigitalInput(8);

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

        setMotorNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Constructs a TalonFXConfig with a default PID.
     */
    private static TalonFXConfiguration getDefaultTalonFXConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = .7;
        config.Slot0.kI = .5;
        config.Slot0.kD = .1;
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
        if (newLevel == null || goToLevel.equals(newLevel) || notInitialized())
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
        System.out.println(bottomLimitSwitch.get());
        if (isChangingLevel) {
            // Doesn't move the elevator if the elevator is not initialized and the robot is
            // already at the bottom.
            if (!(notInitialized() && bottomLimitSwitch.get()))
                moveElevator();
            // The elevator is already moving, default checks can now continue.
            isChangingLevel = false;
            /*
             * If the home position is not the level that the elevator should be going to,
             * then 5 cycles of recalibrating the home positions (~100 ms) are skipped to
             * ensure enough time for the elevator to deactivate the bottom limit switch
             * and not trigger a recalibration stop in case the elevator is at the bottom
             * currently.
             * skipCycles is set to 0 if the elevator is going to the home position.
             */
            if (!ElevatorLevels.HOME.equals(goToLevel))
                skipCycles = 5;
            else
                skipCycles = 0;
        } else
            defaultPeriodic();
        scheduleUpdates();
    }

    /**
     * Schedules updates for all the helpers.
     */
    private void scheduleUpdates() {
        for (Helper helper : helpers)
            helper.scheduleUpdate();
    }

    /**
     * Checks if the elevator is not initialized
     * 
     * @return true if the elevator is not initialized, false if the elevator is
     *         initialized.
     */
    private boolean notInitialized() {
        return leftHomePos == Double.MAX_VALUE || rightHomePos == Double.MAX_VALUE;
    }

    /**
     * The default periodic method for the elevator.
     * This includes recalibration of the home positions and moving the elevator
     * down if the home positions are not initialized.
     */
    private void defaultPeriodic() {
        /**
         * If the elevator is not braking (was moving), the bottom limit switch is
         * pressed, and either the elevator is not initialized or this check not
         * intentionally skipped, then stop the elevator and update the home positions.
         */
        if (!isBraked && skipCycles == 0 && bottomLimitSwitch.get()) {
            stopMotors();
            setHomePositions(leftPos.get(), rightPos.get());
            return;
        } else if (skipCycles > 0)
            skipCycles--;

        if (ElevatorLevels.HOME.equals(goToLevel) && !bottomLimitSwitch.get()) {
            moveDown();
            return;
        }

        /**
         * If the elevator is not braking (was moving) and the difference
         * between the current elevator position and the goToLevel position is less than
         * 1, stop the motors.
         */
        if (!isBraked && !ElevatorLevels.HOME.equals(goToLevel)
                && Math.abs(leftPos.get() - (encoderGetter.get(goToLevel) + leftHomePos)) <= 0.1) {
            stopMotors();
            return;
        }
    }

    /**
     * Moves the elevator down.
     */
    public void moveDown() {
        double speed;
        if (leftHomePos == Double.MAX_VALUE)
            speed = -0.05;
        else
            speed = Math.min(-0.175 * (((double) leftPos.get() + leftHomePos)
                    / encoderGetter.get(ElevatorLevels.values()[ElevatorLevels.values().length - 1])), -0.035);
        setMotorNeutralMode(NeutralModeValue.Coast);
        elevatorLeftMotor.set(speed);
        elevatorRightMotor.set(speed);
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
        return bottomLimitSwitch.get();
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
        if (!ElevatorLevels.HOME.equals(goToLevel))
            setMotorPositions(encoderGetter.get(goToLevel));
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
    }

    /**
     * Sets the neutral mode of both elevator motors.
     * Updates the isBraked variable accordingly.
     * 
     * @param modeValue is the neutral mode of the motors.
     */
    private void setMotorNeutralMode(NeutralModeValue modeValue) {
        elevatorLeftMotor.setNeutralMode(modeValue);
        elevatorRightMotor.setNeutralMode(modeValue);
        isBraked = NeutralModeValue.Brake.equals(modeValue);
    }
     /*
     * Attempts to change the level of the elevator.
     * @param increaseElevator is true if the elevator should increase in level, false if the elevator should decrease in level.
     * @return true if the elevator can change its level, false otherwise.
     */
    public boolean changeLevel(boolean increaseElevator)
    {
        throw new UnsupportedOperationException("Unimplemented method 'changeLevel'");
    }

    /**
     * Sets the positions of both elevator motors.
     * 
     * @param position is the position to set the motors to.
     */
    private void setMotorPositions(double position) {
        elevatorLeftMotor.setControl(LEFT_POS_VOLTAGE.withPosition(leftHomePos + position));
        elevatorRightMotor.setControl(RIGHT_POS_VOLTAGE.withPosition(rightHomePos + position));
    }
}