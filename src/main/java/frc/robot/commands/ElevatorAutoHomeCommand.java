package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorAutoHomeCommand extends Command{

    private ElevatorSubsystem _ElevatorSubsystem;
    
    public ElevatorAutoHomeCommand(ElevatorSubsystem ElevatorSubsystem){
        _ElevatorSubsystem = ElevatorSubsystem;
        addRequirements(_ElevatorSubsystem);
    }

    @Override
    public void initialize() {
        _ElevatorSubsystem.elevatorLeftMotor.set(-0.1);
        _ElevatorSubsystem.elevatorRightMotor.set(-0.1);
    }

    @Override
    public boolean isFinished() {
        return !_ElevatorSubsystem.bottomlimitSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        _ElevatorSubsystem.elevatorLeftMotor.set(0.0);
        _ElevatorSubsystem.elevatorRightMotor.set(0.0);

        StatusSignal<Angle> leftPos = _ElevatorSubsystem.elevatorLeftMotor.getPosition();
        StatusSignal<Angle> rightPos = _ElevatorSubsystem.elevatorRightMotor.getPosition();

        leftPos.waitForUpdate(1.0);
        rightPos.waitForUpdate(1.0);

        _ElevatorSubsystem.leftHomePos = leftPos.getValueAsDouble();
        _ElevatorSubsystem.rightHomePos = rightPos.getValueAsDouble();
    }

}
