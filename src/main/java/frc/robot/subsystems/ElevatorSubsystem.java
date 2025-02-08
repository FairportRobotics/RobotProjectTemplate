package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.commands.ElevatorAutoHomeCommand;
import frc.robot.commands.ElevatorGoToLevelCommand;

public class ElevatorSubsystem extends SubsystemBase {

    private double leftHomePos = Double.MAX_VALUE;
    private double rightHomePos = Double.MAX_VALUE;

    private TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorMotors.LEFT_ID);
    private TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorMotors.RIGHT_ID);
    //DigitalInput toplimitSwitch;
    private DigitalInput bottomlimitSwitch;

    private ElevatorLevels goToLevel = Constants.ElevatorLevels.HOME;

    private StatusSignal<Angle> leftPos;
    private StatusSignal<Angle> rightPos;

    private boolean isChangingLevel = false, isCalibrated = false;

    private int skipCycles;

    private ElevatorAutoHomeCommand autoHomeCommand;

    public ElevatorSubsystem() {
        //toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(Constants.ElevatorLimitSwitches.BOTTOM_ID);

        TalonFXConfiguration elevatorMotor1Config = getDefaultTalonFXConfiguration();
        elevatorMotor1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);
        leftPos = elevatorLeftMotor.getPosition();
        leftPos.setUpdateFrequency(50);
        elevatorLeftMotor.optimizeBusUtilization();
        //elevatorMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration elevatorMotor2Config = getDefaultTalonFXConfiguration();
        elevatorMotor2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorRightMotor.getConfigurator().apply(elevatorMotor2Config);
        rightPos = elevatorRightMotor.getPosition();
        rightPos.setUpdateFrequency(50);
        elevatorRightMotor.optimizeBusUtilization();
        //elevatorMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    }

    private static TalonFXConfiguration getDefaultTalonFXConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 1.7;
        config.Slot0.kI = 1.5;
        config.Slot0.kD = 0.3;
        return config;
    }

    public ElevatorLevels getGoToLevel(){
        return goToLevel;
    }

    public void setLevel(ElevatorLevels newLevel){
        if(newLevel == null || goToLevel.equals(newLevel))
            return;
        goToLevel = newLevel;
        //The elevator must change its position now.
        isChangingLevel = true;
    }

    /**
     * (This the initializer of the elevator subsystem when the robot is enabled)
     * Moves the elevator down while the leftHomePos or rightHomePos are not set.
     * When the bottom limit switch is pressed, leftHomePos and rightHomePos are initialized and the motors are stopped.
     */
    @Override
    public void periodic() {
        System.out.println(goToLevel.toString());
        if(isChangingLevel)
        {
            //Moves the elevator to the level
            moveElevator();
            //The elevator is already moving, default checks can now continue.
            isChangingLevel = false;
            //The home position will recalibrate the next time the elevator reaches home position.
            isCalibrated = false;
            //Skips 5 cycles of recalibrating the home positions (~100 ms).
            skipCycles = 5;
        }
        else
            defaultPeriodic();
    }

    private void defaultPeriodic()
    {
        /**
         * If the bottom limit switch is pressed, the home positions are not already calibrated, and this check is not intentially skipped, recalibrate the home position.
         */
        if (!bottomlimitSwitch.get() && !isCalibrated && skipCycles == 0)
        {
            elevatorLeftMotor.set(0.0);
            elevatorRightMotor.set(0.0);

            StatusSignal<Angle> leftPos = elevatorLeftMotor.getPosition();
            StatusSignal<Angle> rightPos = elevatorRightMotor.getPosition();

            leftPos.waitForUpdate(1.0);
            rightPos.waitForUpdate(1.0);
    
            leftHomePos = leftPos.getValueAsDouble();
            rightHomePos = rightPos.getValueAsDouble();

            elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
            elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);

            //The home positions are now calibrated
            isCalibrated = true;
            return;
        }

        if(skipCycles > 0)
            skipCycles--;

        /**
         * If robot home position not initialized, keep moving down.
         */
        if(leftHomePos == Double.MAX_VALUE || rightHomePos == Double.MAX_VALUE)
        {
            set(-0.1);
            return;
        }
    }

    /**
     * Gets the left motor of the elevator.
     * @return the left motor of the elevator.
     */
    public TalonFX getLeftMotor(){
        return elevatorLeftMotor;
    }

    /**
     * Gets the right motor of the elevator.
     * @return the right motor of the elevator.
     */
    public TalonFX getRightMotor(){
        return elevatorRightMotor;
    }

    /**
     * Sets the speed of the elevator motors.
     * @param speed is the speed of the motors to be set to.
     */
    public void set(double speed)
    {
        elevatorLeftMotor.set(speed);
        elevatorRightMotor.set(speed);
    }

    /**
     * Gets the bottom limit switch of the elevator as a boolean.
     * @return true if the bottom limit switch is pressed, false otherwise.
     */
    public boolean getBottomLimitSwitchAsBoolean()
    {
        return bottomlimitSwitch.get();
    }

    /**
     * Gets the home position of the left motor.
     * @return the the home position of the left motor.
     */
    public double getLeftHomePos(){
        return leftHomePos;
    }

    /**
     * Gets the home position of the right motor.
     * @return the the home position of the right motor.
     */
    public double getRightHomePos(){
        return rightHomePos;
    }

    /**
     * Sets the home position of the left motor.
     * @param pos is the position to set the home position to.
     */
    public void setLeftHomePos(double pos){
        leftHomePos = pos;
    }

    /**
     * Sets the home position of the right motor.
     * @param pos is the position to set the home position to.
     */
    public void setRightHomePos(double pos){
        rightHomePos = pos;
    }

    private void moveElevator() {
        setMotorNeutralMode(NeutralModeValue.Coast);
        setMotorPositions(ElevatorGoToLevelCommand.getEncoderValueForLevel(goToLevel));
    }

    private void setMotorNeutralMode(NeutralModeValue modeValue)
    {
        elevatorLeftMotor.setNeutralMode(modeValue);
        elevatorRightMotor.setNeutralMode(modeValue);
    }

    private void setMotorPositions(double position)
    {
        elevatorLeftMotor.setControl(new PositionVoltage(leftHomePos + position));
        elevatorRightMotor.setControl(new PositionVoltage(rightHomePos + position));
    }
}
