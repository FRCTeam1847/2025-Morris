// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Light extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int bufferSize = Constants.ConfingValues.LIGHTSBUFFERSIZE;
  private int knightIndex = 0;
  private int knightDelayCounter = 0;
  private static final int KNIGHT_DELAY_CYCLES = 3; // Adjust to make it slower (higher = slower)

  /** Creates a new Light. */
  public Light() {
    led = new AddressableLED(Constants.ConfingValues.LightsPWMPORT);
    ledBuffer = new AddressableLEDBuffer(bufferSize);
    led.setLength(bufferSize);

    // setDefaultCommand(ru);
    setDefaultCommand(runPattern(LEDPattern.solid(Color.kWhite)).withName("lightsOn"));
    led.start();
  }

  public Command runPattern(LEDPattern pattern) {
    System.out.println("setting lights on");
    return run(() -> pattern.applyTo(ledBuffer));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!DriverStation.isEnabled()) {
      boolean targetVisible = LimelightHelpers.getTV("limelight-main");
      if (targetVisible) {
        // Solid Green when disabled and can see an april tag
        for (int i = 0; i < bufferSize; i++) {
          ledBuffer.setRGB(i, 0, 255, 0);
        }
      } else if (knightDelayCounter == 0) { // Loading spinner
        // clear all
        for (int i = 0; i < bufferSize; i++) {
          ledBuffer.setRGB(i, 0, 0, 0);
        }
        if (DriverStation.isDSAttached()) {
          // Connected but looking for april tag
          ledBuffer.setRGB(knightIndex, 255, 0, 0);
        } else {
          // Not connected to driver station
          ledBuffer.setRGB(knightIndex, 255, 0, 0);
        }

        // advance and wrap
        knightIndex = (knightIndex + 1) % bufferSize;

        knightDelayCounter = KNIGHT_DELAY_CYCLES; // higher = slower
      } else {
        knightDelayCounter--;
      }
    } else {
      /** Enabled show white light */

      // boolean targetVisible = LimelightHelpers.getTV("");
      // if (targetVisible) {
      for (int i = 0; i < bufferSize; i++) {
        ledBuffer.setRGB(i, 255, 255, 255);
      }
      // } else {
      // for (int i = 0; i < bufferSize; i++) {
      // ledBuffer.setRGB(i, 0, 255, 0);
      // }
      // }
    }
    led.setData(ledBuffer);
  }

}
