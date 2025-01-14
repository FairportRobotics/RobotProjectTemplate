package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {
    public final TalonSRX motor = new TalonSRX(17);
    private boolean motorActivated;

    /**
     * Constructor of the Test Subsystem
     * @param motorActivated is true when the motor should be on at that time, false otherwise.
     */
    public TestSubsystem(boolean motorActivated)
    {
        this.motorActivated = motorActivated;
    }

    /**
     * Runs the motor depending on if the motor should be running at that time during construction.
     */
    public void runMotor() {
        if(motorActivated)
            motor.set(ControlMode.PercentOutput, .1);
        else
            motor.set(ControlMode.PercentOutput, 0);
    }
}