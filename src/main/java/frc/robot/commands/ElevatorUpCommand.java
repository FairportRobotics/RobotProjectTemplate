package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUpCommand extends Command{

    private ElevatorSubsystem _ElevatorSubsystem;
    private double requestPosRots = Double.MAX_VALUE;

    private StatusSignal<Angle> leftPosition;
    private StatusSignal<Angle> rightPosition;

    private StatusSignal<Double> leftPosError;
    private StatusSignal<Double> rightPosError;

    final PositionVoltage rightPositionRequest;
    final PositionVoltage leftPositionRequest;

    public ElevatorLevels pos;

    public ElevatorUpCommand(ElevatorSubsystem ElevatorSubsystem){
        pos = ElevatorSubsystem.getLevel();
        if (pos != ElevatorLevels.FOUR) {
            pos = ElevatorLevels.values()[pos.ordinal() + 1];
        }
        _ElevatorSubsystem = ElevatorSubsystem;
        addRequirements(_ElevatorSubsystem);

        requestPosRots = getEncoderValueForLevel(pos);

        rightPosition = _ElevatorSubsystem.elevatorRightMotor.getPosition();
        leftPosition = _ElevatorSubsystem.elevatorLeftMotor.getPosition();

        leftPosError = _ElevatorSubsystem.elevatorLeftMotor.getClosedLoopError();
        rightPosError = _ElevatorSubsystem.elevatorRightMotor.getClosedLoopError();

        rightPositionRequest = new PositionVoltage(0).withSlot(0);
        leftPositionRequest = new PositionVoltage(0).withSlot(0);
    }

    @Override
    public void initialize() {
        _ElevatorSubsystem.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Coast);
        _ElevatorSubsystem.elevatorRightMotor.setNeutralMode(NeutralModeValue.Coast);

        _ElevatorSubsystem.elevatorLeftMotor
                .setControl(leftPositionRequest.withPosition(_ElevatorSubsystem.leftHomePos + requestPosRots));
        _ElevatorSubsystem.elevatorRightMotor
                .setControl(rightPositionRequest.withPosition(_ElevatorSubsystem.rightHomePos + requestPosRots));
    }

    @Override
    public boolean isFinished() {
        leftPosError.refresh();
        rightPosError.refresh();

        if (requestPosRots <= 0) {
            return !_ElevatorSubsystem.bottomlimitSwitch.get();
        } else if (leftPosition.hasUpdated() && rightPosition.hasUpdated()) {

            SmartDashboard.putNumber("Ele Left Pos", leftPosition.getValueAsDouble());
            SmartDashboard.putNumber("Ele Right", rightPosition.getValueAsDouble());

            return (Math.abs(leftPosition.getValueAsDouble() - (requestPosRots + _ElevatorSubsystem.leftHomePos)) <= 0.1
                    ||
                    Math.abs(rightPosition.getValueAsDouble()
                            - (requestPosRots + _ElevatorSubsystem.rightHomePos)) <= 0.1);
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // RobotContainer.noteAquired = false;
        _ElevatorSubsystem.elevatorLeftMotor.stopMotor();
        _ElevatorSubsystem.elevatorRightMotor.stopMotor();
        _ElevatorSubsystem.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        _ElevatorSubsystem.elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public static double getEncoderValueForLevel(ElevatorLevels level) {
        switch (level) {
            case HOME:
                return 0.0;
            case ONE:
                return 0.0;
            case TWO:
                return 0.0;
            case THREE:
                return 0.0;
            case FOUR:
                return 0.0;
            default:
                throw new IllegalArgumentException("Unknown level: " + level);
        }
    }
}
