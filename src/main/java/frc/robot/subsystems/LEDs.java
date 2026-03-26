// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private AddressableLED LEDs;
  private AddressableLEDBuffer LEDBuffer;
  private final int LEDLength = 69; //nice
  private final int pwmPort = 0;

  /** Creates a new LEDs. */
  public LEDs() {
    LEDs = new AddressableLED(pwmPort);

    LEDBuffer = new AddressableLEDBuffer(LEDLength);
    LEDs.setLength(LEDBuffer.getLength());
    
    LEDs.start();

    Distance kLedSpacing = Meters.of((double) 1.0 / LEDLength);

    //LED color pallete
    //solid
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern orange = LEDPattern.solid(Color.kOrange);
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    LEDPattern gold = LEDPattern.solid(Color.kGold);
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    LEDPattern purple = LEDPattern.solid(Color.kPurple);

    //animated & multi
    LEDPattern rainbow = LEDPattern.rainbow(255,255);
    LEDPattern epicBlurpleColor = LEDPattern.steps(Map.of(0, Color.kBlue, 0.5, Color.kPurple));
    LEDPattern epicerBlurpleColor = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlue, Color.kPurple);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (true){
      epicerBlurpleColor.applyTo(LEDBuffer);
      LEDs.setData(LEDBuffer);
    } else {
      red.applyTo(LEDBuffer);
      LEDs.setData(LEDBuffer);
    }
    //no booleans that check if something (like intake) is running :(
  }
}
