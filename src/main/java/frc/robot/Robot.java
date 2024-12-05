// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private AddressableLED leds;
  private AddressableLEDBuffer buffer;
  private Timer timer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // Initialize LEDs
    leds = new AddressableLED(9); // PWM port 9
    buffer = new AddressableLEDBuffer(138); // 138 LEDs
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();

    // Initialize timer for animation
    
  }

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void disabledInit() {
    timer = new Timer();
    timer.start();
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().run();

    // Get the current time in seconds
    double time = timer.get();
    int length = buffer.getLength();

    for (int i = 0; i < length; i++) {
      // Spread hues across LEDs and animate over time
      int hue = (int) ((time * 100 + (i * 360.0 / length)) % 360); // Adjust time multiplier for speed
      int saturation = 255; // Full saturation
      int value = 50;      // Full brightness
      buffer.setLED(i, Color.fromHSV(hue, saturation, value));
    }

    // Push updated LED data to the strip
    leds.setData(buffer);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // for (int i = 0; i<buffer.getLength(); i++) {
    //   buffer.setLED(i, Color.kBlack);
    // }
    // leds.setData(buffer);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
