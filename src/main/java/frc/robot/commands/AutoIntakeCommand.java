package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeCommand extends Command {
    private final HandSubsystem m_subsystem;

    /**
     * Constructs an instance of the AutoIntakeCommand.
     * @param HandSubsystem is the arm subsystem the command is using.
     */ 
    public AutoIntakeCommand(HandSubsystem HandSubsystem)
    {
        m_subsystem = HandSubsystem;

    }

    /**
     * Starts up the arm motor.
     */
    @Override
    public void execute()
    {
        //HandSubsystem.armYMotor.setNeutralMode(NeutralModeValue.Coast);
        m_subsystem.setSpeed(1);
    }

    /**
     * The command is finished when the limit switch is pressed.
     * @return true if the command is finished, false otherwise.
     */
    @Override
    public boolean isFinished()
    {
        return !m_subsystem.getSwitch();
    }

    /**
     * Stops the motor
     * @param interrupted is true if the command was interrupted.
     */
    @Override
    public void end(boolean interrupted)
    {
        m_subsystem.setSpeed(0);
    }
}
