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
import frc.robot.Constants;
import frc.robot.Constants.Levels;

public class IntakeSubsystem extends SubsystemBase {
  private LaserCan outerLaserCAN;
  private LaserCan innerLaserCAN;
  private final SparkMax intakeMotor;
  private double intakeSpeed = 0;

  private enum IntakeState {
    IDLE,
    RUNNING
  }

  private IntakeState state = IntakeState.IDLE;

  private final Timer simulationTimer = new Timer();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.smartCurrentLimit(20);
    intakeMotor = new SparkMax(Constants.ConfingValues.IntakeCANID, MotorType.kBrushless);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    outerLaserCAN = new LaserCan(Constants.ConfingValues.OuterLaserCANCANID);
    innerLaserCAN = new LaserCan(Constants.ConfingValues.InnerLaserCANID);
    try {
      outerLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      outerLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      outerLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
    try {
      innerLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      innerLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      innerLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public void intake() {
    intakeSpeed = Constants.INTAKESPEED;
  }

  public void release(Levels level) {
    if (level == Levels.L1) {
      intakeSpeed = Constants.L1RELEASESPEED;
    } else {
      intakeSpeed = Constants.RELEASESPEED;
    }
  }

  public void stopIntake() {
    intakeSpeed = 0;
  }

  public void resetSimulationTimer() {
    if (!RobotBase.isSimulation()) {
      return;
    }
    simulationTimer.reset();
    simulationTimer.start();
  }

  public boolean isOuterSensorTriggered() {
    if (RobotBase.isSimulation()) {
      boolean triggered = simulationTimer.hasElapsed(2);
      return triggered;
    } else {
      LaserCan.Measurement measurement = outerLaserCAN.getMeasurement();
      // if (measurement != null && measurement.status ==
      // LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      // System.out.println("The target is " + measurement.distance_mm + "mm away!");
      // }
      return (measurement != null &&
          measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
          measurement.distance_mm < 100);
    }
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> resetSimulationTimer(), this)
        .andThen(new RunCommand(() -> intake(), this))
        .until(this::isOuterSensorTriggered)
        .andThen(new InstantCommand(() -> stopIntake(), this));
  }

  public Command intakeCommandUntilCoral() {
    return new RunCommand(() -> System.out.println("Intake!!!"), this).until(this::isInnerSensorTriggered);
  }

  public boolean isInnerSensorTriggered() {
    if (RobotBase.isSimulation()) {
      return true; // or simulate a delay like in `isSensorTriggered`
    } else {
      LaserCan.Measurement measurement = innerLaserCAN.getMeasurement();
      return (measurement != null &&
          measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
          measurement.distance_mm < 50); // adjust threshold as needed
    }
  }

  @Override
  public void periodic() {
    switch (state) {
      case IDLE:
        // Only start if inner is triggered and lc is NOT triggered
        if (isInnerSensorTriggered() && !isOuterSensorTriggered()) {
          intake();
          state = IntakeState.RUNNING;
        }
        break;

      case RUNNING:
        // Stop if lc is triggered (we've passed the outer sensor)
        if (isOuterSensorTriggered()) {
          stopIntake();
          state = IntakeState.IDLE;
        }
        break;
    }
    ; // stop only if we've started already

    intakeMotor.set(intakeSpeed);
    Logger.recordOutput("Field/Robot/ManipulatorMechanism/intake", intakeSpeed);
    Logger.recordOutput("Field/Robot/ManipulatorMechanism/intakeOuterSensor", isOuterSensorTriggered());
    Logger.recordOutput("Field/Robot/ManipulatorMechanism/intakeInnerSensor", isInnerSensorTriggered());
  }
}
