package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeCommand extends Command {
    private final HandSubsystem h_Subsystem;

    /**
     * Constructs an instance of the AutoIntakeCommand.
     * @param armSubsystem is the arm subsystem the command is using.
     */ 
    public AutoIntakeCommand(HandSubsystem HandSubsystem)
    {
        h_Subsystem = HandSubsystem;

    }

    /**
     * Starts up the arm motor.
     */
    @Override
    public void execute()
    {
        HandSubsystem.armYMotor.setNeutralMode(NeutralModeValue.Coast);
        HandSubsystem.armYMotor.set(0.1);
    }

    /**
     * The command is finished when the limit switch is pressed.
     * @return true if the command is finished, false otherwise.
     */
    @Override
    public boolean isFinished()
    {
        return !h_Subsystem.getSwitch();
    }

    /**
     * Stops the motor
     * @param interrupted is true if the command was interrupted.
     */
    @Override
    public void end(boolean interrupted)
    {
        h_Subsystem.armYMotor.set(0);
    }
}
