// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.*;

public class LEDs extends SubsystemBase{
  private AddressableLED LED;
  private AddressableLEDBuffer LEDBuffer;
  private AddressableLEDBufferView leftLEDBufferView;
  private AddressableLEDBufferView rightLEDBufferView;
  private final int leftLEDLength = 36;   //should be the correct length as of 3/27/26
  private final int rightLEDLength = 37;  //should be the correct length as of 3/27/26
  private final int pwmPort = 9;

  public LEDs() {
      //set pwmPort
      LED = new AddressableLED(pwmPort);

      //set strip length
      LEDBuffer = new AddressableLEDBuffer(leftLEDLength + rightLEDLength);
      leftLEDBufferView = LEDBuffer.createView(0, leftLEDLength);
      rightLEDBufferView = LEDBuffer.createView(leftLEDLength, rightLEDLength).reversed();
      LED.setLength(LEDBuffer.getLength());

      //start LEDs
      LED.start();
    }

  //LEDs per Meter
  Distance kLedSpacing = Meters.of((double) 1 / (leftLEDLength + rightLEDLength));

  //create color palate
  //Solid colors
  LEDPattern red = LEDPattern.solid(Color.kRed);
  LEDPattern blue = LEDPattern.solid(Color.kBlue);
  LEDPattern green = LEDPattern.solid(Color.kGreen);
  LEDPattern yellow = LEDPattern.solid(Color.kYellow);
  LEDPattern gold = LEDPattern.solid(Color.kGold);
  LEDPattern orange = LEDPattern.solid(Color.kOrange);
  LEDPattern purple = LEDPattern.solid(Color.kPurple);
  LEDPattern manualYellow = LEDPattern.solid(new Color(255, 135, 0));
  LEDPattern manualCyan = LEDPattern.solid(new Color(0, 255, 225));
  LEDPattern breathingManualYellow = manualYellow.breathe(Time.ofRelativeUnits(3, Seconds.getBaseUnit()));
  LEDPattern manualGreen = LEDPattern.solid(new Color(0, 255, 0));
  LEDPattern flashPurple = purple.blink(Time.ofRelativeUnits(0.1, Seconds.getBaseUnit()));
  LEDPattern flashGreen = manualGreen.blink(Time.ofRelativeUnits(0.1, Seconds.getBaseUnit()));

  //animated colors
  LEDPattern rainbow = LEDPattern.rainbow(255, 255);
  LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
  LEDPattern redAndBlue = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));
  LEDPattern timeProgress = LEDPattern.progressMaskLayer(() -> DriverStation.getMatchTime() / 135);

  double startingPercent = 0;
  double endingPercent = 0.25;
  public LEDPattern yellowChase(double startingPercent, double endingPercent) {
    startingPercent = startingPercent % 1;
    endingPercent = endingPercent % 1;
    return LEDPattern.steps(Map.of(startingPercent, new Color(255, 135, 0), endingPercent, Color.kBlack));
  }

  @Override
  public void periodic() {
    //yellowChase(startingPercent, endingPercent).applyTo(leftLEDBufferView);
    //yellowChase(startingPercent, endingPercent).applyTo(rightLEDBufferView);
    manualYellow.applyTo(LEDBuffer);
    LED.setData(LEDBuffer);

    startingPercent += 0.005;
    endingPercent += 0.005;
  }
}
