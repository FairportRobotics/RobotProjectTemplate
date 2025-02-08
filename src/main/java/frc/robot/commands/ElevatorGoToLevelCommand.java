package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToLevelCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;
    protected ElevatorLevels goToLevel;

    private StatusSignal<Angle> leftPosition;
    private StatusSignal<Angle> rightPosition;

    private StatusSignal<Double> leftPosError;
    private StatusSignal<Double> rightPosError;

    final PositionVoltage rightPositionRequest;
    final PositionVoltage leftPositionRequest;

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, ElevatorLevels goToLevel) {
        addRequirements(elevatorSubsystem);
        rightPositionRequest = new PositionVoltage(0).withSlot(0);
        leftPositionRequest = new PositionVoltage(0).withSlot(0);

        this.elevatorSubsystem = elevatorSubsystem;
        this.goToLevel = goToLevel;

        rightPosition = elevatorSubsystem.getRightMotor().getPosition();
        leftPosition = elevatorSubsystem.getLeftMotor().getPosition();

        leftPosError = elevatorSubsystem.getLeftMotor().getClosedLoopError();
        rightPosError = elevatorSubsystem.getRightMotor().getClosedLoopError();
    }


    @Override
    public void initialize() {
        elevatorSubsystem.setLevel(goToLevel);
    }


    @Override
    public boolean isFinished() {
       //return isCloseEnoughToRightLevel();
       return true;
    }

    /**
     * Checks if the elevator position is clos enough to the goToLevel of the elevator.
     * @return true if it is clos enough, false otherwise.
     */
    private boolean isCloseEnoughToRightLevel()
    {
        ErrorRefresh();
        MotorRefresh();
        double error = Math.max(leftPosError.getValueAsDouble(), rightPosError.getValueAsDouble());
        double positionOfElevator = elevatorSubsystem.getLeftMotor().getPosition().getValueAsDouble();
        if(Math.abs(positionOfElevator - getEncoderValueForLevel(elevatorSubsystem.getGoToLevel())) <= Math.max(error, 0.5) ||
        (ElevatorLevels.HOME.equals(elevatorSubsystem.getGoToLevel()) && elevatorSubsystem.getBottomLimitSwitchAsBoolean()))
            return true;
        return false;
    }

    /**
     * Stops the elevator motors when the command ends.
     */
    @Override
    public void end(boolean interrupted) {
        // RobotContainer.noteAquired = false;
    }

    /**
     * Gets the encoder value from Constants.java for the given level.
     * @param level The level to get the encoder value for.
     * @return The encoder value for the given level.
     */
    public static double getEncoderValueForLevel(ElevatorLevels level) {
        switch (level) {
            case HOME:
                return Constants.ElevatorEncoderValues.HOME;
            case CORAL:
                return Constants.ElevatorEncoderValues.CORAL;
            case ONE:
                return Constants.ElevatorEncoderValues.ONE;
            case TWO:
                return Constants.ElevatorEncoderValues.TWO;
            case THREE:
                return Constants.ElevatorEncoderValues.THREE;
            case FOUR:
                return Constants.ElevatorEncoderValues.FOUR;
            //case MAX:
                //return Constants.ElevatorEncoderValues.MAX;
            default:
                throw new IllegalArgumentException("Unknown level: " + level);
        }
    }

    /**
     * Stops the elevator motors.
     */
    private void stopMotors() {
        elevatorSubsystem.getLeftMotor().stopMotor();
        elevatorSubsystem.getRightMotor().stopMotor();
    }

    /**
     * Sets the neutral mode of the elevator motors to brake.
     */
    private void setMotorNeutralMode(NeutralModeValue modeValue) {
        elevatorSubsystem.getLeftMotor().setNeutralMode(modeValue);
        elevatorSubsystem.getRightMotor().setNeutralMode(modeValue);
    }

    /**
     * Refreshes the error signals for the left and right motors.
     */
    private void ErrorRefresh() {
        leftPosError.refresh();
        rightPosError.refresh();
    }

    private void MotorRefresh()
    {
        leftPosition.refresh();
        rightPosition.refresh();
    }
}
