// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  AddressableLED leds;
  AddressableLEDBuffer buffer;
  public Leds(AddressableLED leds, AddressableLEDBuffer buffer) {
    this.leds = leds;
    this.buffer = buffer;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAll(Color color) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color); // Set all LEDs to red
    }
    leds.setData(buffer);
  }
}
