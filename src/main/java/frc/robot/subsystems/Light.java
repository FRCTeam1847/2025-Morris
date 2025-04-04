// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Light extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledbuff;


  /** Creates a new Light. */
  public Light() {
    led = new AddressableLED(9);
    ledbuff = new AddressableLEDBuffer(12);
    led.setLength(12);

    
   // setDefaultCommand(ru);
   setDefaultCommand(runPattern(LEDPattern.solid(Color.kWhite)).withName("lightsOn"));
   led.start();
  }

  public Command runPattern(LEDPattern pattern){
    System.out.println("setting lights on");
    return run(()-> pattern.applyTo(ledbuff));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    led.setData(ledbuff);
  }

}
