
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: Make working algae code 
public class AlgaeSubsystem extends SubsystemBase {
/*I WILL DELETE THIS LATER ITS JUST HERE SO THERE ARENT ANY ERRORS WHEN I PUSH THIS WFILE
  public ExampleSubsystem() 
  {//NAME

    return ; 
  }//NAME
*/
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand()
  {//NAME
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }//NAME

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition()
 {//NAME
    // Query some boolean state, such as a digital sensor.
    return false;
  }//NAME

  @Override
  public void periodic() 
  {//NAME
    // This method will be called once per scheduler run
  }//NAME

  @Override
  public void simulationPeriodic()
  {//NAME
    // This method will be called once per scheduler run during simulation
  }//NAME
}
