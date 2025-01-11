package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestSubsystem extends SubsystemBase {
    public final TalonFX motor = new TalonFX(Constants.MotorConstants.MOTOR_ID);
    public TestSubsystem()
    {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0
    }
}