// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.*;

public class LEDs extends SubsystemBase{
  private final Shooter shooter;
  private final Intake intake;

  private AddressableLED LED;
  private AddressableLEDBuffer LEDBuffer;
  private AddressableLEDBufferView leftLEDBufferView;
  private AddressableLEDBufferView rightLEDBufferView;
  private final int leftLEDLength = 35;   //should be the correct length as of 3/27/26
  private final int rightLEDLength = 36;  //should be the correct length as of 3/27/26
  private final int pwmPort = 9;

  //LEDs per Meter
  Distance kLedSpacing = Meters.of((double) 1 / (leftLEDLength + rightLEDLength));

  //create color palate
  //Solid colors
  LEDPattern black = LEDPattern.solid(Color.kBlack);
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
  LEDPattern flashYellow = manualYellow.blink(Time.ofRelativeUnits(0.1, Seconds.getBaseUnit()));

  //animated colors
  LEDPattern rainbow = LEDPattern.rainbow(255, 255);
  LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
  LEDPattern redAndBlue = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));
  LEDPattern timeProgress = LEDPattern.progressMaskLayer(() -> DriverStation.getMatchTime() / 135);
  LEDPattern shooterProgress;

  double startingPercent = 0;
  double endingPercent = 0.25;

  public LEDs(Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;

    //set pwmPort
    LED = new AddressableLED(pwmPort);

    //set strip length
    LEDBuffer = new AddressableLEDBuffer(leftLEDLength + rightLEDLength);
    leftLEDBufferView = LEDBuffer.createView(0, leftLEDLength-1);
    rightLEDBufferView = LEDBuffer.createView(leftLEDLength, LEDBuffer.getLength()-1).reversed();
    LED.setLength(LEDBuffer.getLength());

    shooterProgress = LEDPattern.progressMaskLayer(() -> shooter.getVelocity()/shooter.getTargetVelocity());

    //start LEDs
    LED.start();
  }


  public LEDPattern yellowChase(double startingPercent, double endingPercent) {
    startingPercent = startingPercent % 1;
    endingPercent = endingPercent % 1;
    return LEDPattern.steps(Map.of(startingPercent, new Color(255, 135, 0), endingPercent, Color.kBlack));
  }

  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return false;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game.
      return false;
    }
  }

  public boolean yellowBlinkActive() {
    boolean shift1To2 = DriverStation.getMatchTime() > 105 && DriverStation.getMatchTime() <= 107;
    boolean shift2To3 = DriverStation.getMatchTime() > 80 && DriverStation.getMatchTime() <= 82;
    boolean shift3To4 = DriverStation.getMatchTime() > 55 && DriverStation.getMatchTime() <= 57;
    return shift1To2 || shift2To3 || shift3To4;
  }

  @Override
  public void periodic() {
    if(DriverStation.isDisabled()) {
      yellowChase(startingPercent, endingPercent).applyTo(leftLEDBufferView);
      yellowChase(startingPercent, endingPercent).applyTo(rightLEDBufferView);

      startingPercent += 0.005;
      endingPercent += 0.005;
    } else {
      if(shooter.getIsShooting()) {
        if(shooter.isAtVelocity()) {
          flashGreen.applyTo(leftLEDBufferView);
          flashGreen.applyTo(rightLEDBufferView);
        } else {
          manualYellow.mask(shooterProgress).applyTo(leftLEDBufferView);
          manualYellow.mask(shooterProgress).applyTo(rightLEDBufferView);
        }
      } else if(intake.getIsIntaking()) {
        flashPurple.applyTo(leftLEDBufferView);
        flashPurple.applyTo(rightLEDBufferView);
      } else if(yellowBlinkActive()) {
        flashYellow.applyTo(leftLEDBufferView);
        flashYellow.applyTo(rightLEDBufferView);
      } else if(isHubActive()) {
        manualYellow.applyTo(leftLEDBufferView);
        manualYellow.applyTo(rightLEDBufferView);  
      }else if(DriverStation.getMatchTime() < 30) {
        scrollingRainbow.applyTo(leftLEDBufferView);
        scrollingRainbow.applyTo(rightLEDBufferView);
      } else {
        black.applyTo(leftLEDBufferView);
        black.applyTo(rightLEDBufferView);
      }
    }
    LED.setData(LEDBuffer);
  }
}
