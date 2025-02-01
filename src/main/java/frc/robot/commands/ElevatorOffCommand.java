package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorOffCommand extends Command{

    ElevatorSubsystem _ElevatorSubsystem;

    public ElevatorOffCommand(ElevatorSubsystem ElevatorSubsystem){
        _ElevatorSubsystem = ElevatorSubsystem;
        addRequirements(_ElevatorSubsystem);
    }

    @Override
    public void initialize() {
        _ElevatorSubsystem.elevatorLeftMotor.set(0);
        _ElevatorSubsystem.elevatorRightMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
