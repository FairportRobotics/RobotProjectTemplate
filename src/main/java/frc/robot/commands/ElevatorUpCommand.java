package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUpCommand extends Command{

    ElevatorSubsystem _ElevatorSubsystem;
    double speed;

    public ElevatorUpCommand(ElevatorSubsystem ElevatorSubsystem, double speed){
        _ElevatorSubsystem = ElevatorSubsystem;
        this.speed = speed;
        addRequirements(_ElevatorSubsystem);
    }

    @Override
    public void initialize() {
        _ElevatorSubsystem.elevatorLeftMotor.set(speed);
        _ElevatorSubsystem.elevatorRightMotor.set(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
