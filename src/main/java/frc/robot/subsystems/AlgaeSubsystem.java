
package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.DIOValues;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;

public class AlgaeSubsystem extends TestableSubsystem
{//START

  //VARIABLES
  private TalonFX fenceMotor; //Brings the entire fence like thing down
  private DigitalInput limitSwitch; //idk where tf this is located lmfao
  private SparkMax wheelSpin; //Spins wheel
  private StatusSignal<Angle> pos;
  //VARIABLES

  /**
   * the system that algaes the systemsub
   */
  public AlgaeSubsystem() 
  {//ALGAESUBSYSTEM
    super("AlgaeSubsystem");
    fenceMotor = new TalonFX(98); // TODO: FIX ID
    wheelSpin = new SparkMax(97, null);//deal with paramaters later
    limitSwitch= new DigitalInput(DIOValues.ALGAELIMIT);
      //PID LOOP
      TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
      krakenConfig.Slot0.kP = 0.8;
      krakenConfig.Slot0.kI = 0.5;
      krakenConfig.Slot0.kD = 0.3;
      //PID LOOP
      fenceMotor.getConfigurator().apply(krakenConfig);
      pos = fenceMotor.getPosition();
      pos.setUpdateFrequency(50);
      fenceMotor.optimizeBusUtilization();
      fenceMotor.getConfigurator().apply(krakenConfig, 0.050);
    
  }//ALGAESUBSYSTEM

   /**
   * Spins the wheels, first line
   * brings down the entire fence
   * Note: last line may change position
   */
  public void openBallIntake()
  {//openBallIntake
    wheelSpin.set(.1); 
    fenceMotor.setNeutralMode(NeutralModeValue.Coast);
    PositionVoltage PosRequest = new PositionVoltage(0).withSlot(0);
    fenceMotor.setControl(PosRequest.withPosition(1)); 
  }//BallIntake

  /**
   * @return Position
  */
  public StatusSignal<Angle> getPos()
  {//GetPos
    return pos;
  }//GetPos
  
  /**
   * Stops the wheels from spinning 
   * Sets the kraken motor speed to -.1 so it closes
  */
  public void closeBallIntake()
  {//closeBallIntake
    wheelSpin.stopMotor();
    fenceMotor.set(-.1);
  }//closeBallIntake

  /**
   * No one will understand what this method does, may contain a dead body :Thumbsup:
  */
  public void stopFenceMotor()
  {//stopFenceMotor
    fenceMotor.stopMotor();
  }//stopFenceMotor

  /**
   * @return Error
  */
  public StatusSignal<Double> getError()
  {//GetError
    return fenceMotor.getClosedLoopError();
  }//GetError

  /**
  */
  public boolean getSwitch()
  {//getSwitch
    return !limitSwitch.get();
  }//getSwitch

  public void periodic() 
  {//Periodic
    if (pos == null) 
    {//if
        fenceMotor.set(-.1);
        if (!limitSwitch.get()) 
        {//Enclosed if
          fenceMotor.set(0.0);
          fenceMotor.setPosition(0);
          pos = fenceMotor.getPosition();
        }//Enclosed if
    }//if
    Logger.recordOutput("Algae at Home ", !limitSwitch.get());  
  }//Periodic

}//END
