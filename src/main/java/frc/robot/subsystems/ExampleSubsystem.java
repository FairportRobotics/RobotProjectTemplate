// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.fairportrobotics.frc.posty.test.BitTest;
import org.fairportrobotics.frc.posty.test.PostTest;
import static org.fairportrobotics.frc.posty.assertions.Assertions.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ExampleSubsystem extends TestableSubsystem {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    super();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @PostTest
  public void myPassingPostTest() {
    assertThat(true).isTrue();
    assertThat(false).isFalse();
  }

  @PostTest(enabled = false)
  public void myFailingPostTest() {
    assertThat("Hello World!").contains("HA");
  }

  @PostTest(name = "Drive Motors Connected", enabled = false)
  public void motorsConnected() {
    assertThat(false).as("Left Motor is not connected").isTrue();
    assertThat(false).as("Right Motor is not connected").isTrue();
  }

  @BitTest
  public void myBitTest() {
  }
}
