package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToLevelCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;
    private double requestPosRots = Double.MAX_VALUE;

    private StatusSignal<Angle> leftPosition;
    private StatusSignal<Angle> rightPosition;

    private StatusSignal<Double> leftPosError;
    private StatusSignal<Double> rightPosError;

    final PositionVoltage rightPositionRequest;
    final PositionVoltage leftPositionRequest;

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, ElevatorLevels elevatorLevel) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);

        requestPosRots = getEncoderValueForLevel(elevatorLevel);

        rightPosition = elevatorSubsystem.elevatorRightMotor.getPosition();
        leftPosition = elevatorSubsystem.elevatorLeftMotor.getPosition();

        leftPosError = elevatorSubsystem.elevatorLeftMotor.getClosedLoopError();
        rightPosError = elevatorSubsystem.elevatorRightMotor.getClosedLoopError();

        rightPositionRequest = new PositionVoltage(0).withSlot(0);
        leftPositionRequest = new PositionVoltage(0).withSlot(0);
    }


    @Override
    public void initialize() {
        setMotorNeutralMode(NeutralModeValue.Coast);

        //Moves both motors toward
        elevatorSubsystem.elevatorLeftMotor
                .setControl(leftPositionRequest.withPosition(elevatorSubsystem.leftHomePos + requestPosRots));
        elevatorSubsystem.elevatorRightMotor
                .setControl(rightPositionRequest.withPosition(elevatorSubsystem.rightHomePos + requestPosRots));
    }

    @Override
    public void execute() {
    }


    @Override
    public boolean isFinished() {
        ErrorRefresh();
        if (requestPosRots <= 0) {
            return !elevatorSubsystem.bottomlimitSwitch.get();
        } else if (leftPosition.hasUpdated() && rightPosition.hasUpdated()) {

            SmartDashboard.putNumber("Ele Left Pos", leftPosition.getValueAsDouble());
            SmartDashboard.putNumber("Ele Right", rightPosition.getValueAsDouble());

            return (Math.abs(leftPosition.getValueAsDouble() - (requestPosRots + elevatorSubsystem.leftHomePos)) <= 0.1
                    ||
                    Math.abs(rightPosition.getValueAsDouble()
                            - (requestPosRots + elevatorSubsystem.rightHomePos)) <= 0.1);
        }
        return false;
    }

    /**
     * Stops the elevator motors when the command ends.
     */
    @Override
    public void end(boolean interrupted) {
        // RobotContainer.noteAquired = false;
        stopMotors();
        setMotorNeutralMode(NeutralModeValue.Brake);
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
            case MAX:
                return Constants.ElevatorEncoderValues.MAX;
            default:
                throw new IllegalArgumentException("Unknown level: " + level);
        }
    }

    /**
     * Stops the elevator motors.
     */
    private void stopMotors() {
        elevatorSubsystem.elevatorLeftMotor.stopMotor();
        elevatorSubsystem.elevatorRightMotor.stopMotor();
    }

    /**
     * Sets the neutral mode of the elevator motors to brake.
     */
    private void setMotorNeutralMode(NeutralModeValue modeValue) {
        elevatorSubsystem.elevatorLeftMotor.setNeutralMode(modeValue);
        elevatorSubsystem.elevatorRightMotor.setNeutralMode(modeValue);
    }

    /**
     * Refreshes the error signals for the left and right motors.
     */
    private void ErrorRefresh() {
        leftPosError.refresh();
        rightPosError.refresh();
    }
}
