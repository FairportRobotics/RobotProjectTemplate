package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.commands.ElevatorAutoHomeCommand;

public class ElevatorSubsystem extends SubsystemBase {

    public double rightHomePos = Double.MAX_VALUE;
    public double leftHomePos = Double.MAX_VALUE;

    public TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_LEFT_MOTOR_ID);
    public TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_RIGHT_MOTOR_ID);
    //DigitalInput toplimitSwitch;
    public DigitalInput bottomlimitSwitch;

    private ElevatorLevels level;

    StatusSignal<Angle> leftPos;
    StatusSignal<Angle> rightPos;

    ElevatorAutoHomeCommand autoHomeCommand;

    public ElevatorSubsystem() {
        //toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_BOTTOM_SWITCH_ID);

        TalonFXConfiguration elevatorMotor1Config = getDefaultTalonFXConfiguration();
        elevatorMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);
        leftPos = elevatorLeftMotor.getPosition();
        leftPos.setUpdateFrequency(50);
        elevatorLeftMotor.optimizeBusUtilization();
        //elevatorMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration elevatorMotor2Config = getDefaultTalonFXConfiguration();
        elevatorMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorRightMotor.getConfigurator().apply(elevatorMotor2Config);
        rightPos = elevatorRightMotor.getPosition();
        rightPos.setUpdateFrequency(50);
        elevatorRightMotor.optimizeBusUtilization();
        //elevatorMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    }

    private static TalonFXConfiguration getDefaultTalonFXConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.7;
        config.Slot0.kI = 0.5;
        config.Slot0.kD = 0.1;
        return config;
    }

    public ElevatorLevels getLevel(){
        return level;
    }

    public void setLevel(ElevatorLevels newLevel){
        level = newLevel;
    }

    /**
     * (This the initializer of the elevator subsystem when the robot is enabled)
     * Moves the elevator down while the leftHomePos or rightHomePos are not set.
     * When the bottom limit switch is pressed, leftHomePos and rightHomePos are initialized and the motors are stopped.
     */
    @Override
    public void periodic() {
        if(leftHomePos == Double.MAX_VALUE || rightHomePos == Double.MAX_VALUE){
            
            this.elevatorLeftMotor.set(-0.1);
            this.elevatorRightMotor.set(-0.1);
            
            if (!this.bottomlimitSwitch.get()) {
                this.elevatorLeftMotor.set(0.0);
                this.elevatorRightMotor.set(0.0);
    
                StatusSignal<Angle> leftPos = elevatorLeftMotor.getPosition();
                StatusSignal<Angle> rightPos = elevatorRightMotor.getPosition();

                leftPos.waitForUpdate(1.0);
                rightPos.waitForUpdate(1.0);
        
                leftHomePos = leftPos.getValueAsDouble();
                rightHomePos = rightPos.getValueAsDouble();

                this.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
                this.elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
            }
        }

        Logger.recordOutput("Elevator At Bottom", !bottomlimitSwitch.get());

        Logger.recordOutput("Elevator Left Pos", leftPos.refresh().getValue());
        Logger.recordOutput("Elevator Right Pos", rightPos.refresh().getValue());
    }
}
