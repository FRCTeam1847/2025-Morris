// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private LaserCan lc;
  private final SparkMax intakeMotor;
  private final double speed = 0.25;
  private double intakeSpeed = 0;

  private final Timer simulationTimer = new Timer();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.smartCurrentLimit(20);
    intakeMotor = new SparkMax(12, MotorType.kBrushless);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    lc = new LaserCan(0);
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public void intake() {

    intakeSpeed = -0.13;

  }

  public void release() {
    System.out.println("Release");
    intakeSpeed = -0.5;
  }

  public void stopIntake() {
    intakeSpeed = 0;
  }

  public void resetSimulationTimer() {
    simulationTimer.reset();
    simulationTimer.start();
  }

  public boolean isSensorTriggered() {
    if (RobotBase.isSimulation()) {
      boolean triggered = simulationTimer.hasElapsed(0.75);
      //System.out.println("Simulation mode: time = " + simulationTimer.get() + ", triggered = " + triggered);
      return triggered;
    } else {
      LaserCan.Measurement measurement = lc.getMeasurement();
      // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      //   System.out.println("The target is " + measurement.distance_mm + "mm away!");
      // }
      return (measurement != null &&
          measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
          measurement.distance_mm < 100);
    }
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> resetSimulationTimer(), this)
        .andThen(new RunCommand(() -> intake(), this))
        .until(this::isSensorTriggered)
        .andThen(new InstantCommand(() -> stopIntake(), this));
  }

  @Override
  public void periodic() {
    intakeMotor.set(intakeSpeed);
    Logger.recordOutput("Field/Robot/ManipulatorMechanism/intake", intakeSpeed);
    Logger.recordOutput("Field/Robot/ManipulatorMechanism/intakeSensor", isSensorTriggered());
  }
}
