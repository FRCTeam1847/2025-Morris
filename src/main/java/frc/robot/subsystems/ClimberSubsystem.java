// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX m_climberMotor = new TalonFX(Constants.ConfingValues.ClimberCANID, "rio");
  private final TalonFX m_intakeMotor = new TalonFX(Constants.ConfingValues.ClimberIntakeCANID, "rio");

  private LaserCan floorLaserCAN;
  private LaserCan climberLaserCan;

  private final DutyCycleEncoder climberEncoder = new DutyCycleEncoder(8);

  public ClimberSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
        .withPeakReverseTorqueCurrent(Amps.of(-120));
    m_climberMotor.getConfigurator().apply(configs);
    m_intakeMotor.getConfigurator().apply(configs);
    climberEncoder.setDutyCycleRange(0, 360);

    floorLaserCAN = new LaserCan(Constants.ConfingValues.FloorLaserCANCANID);
    try {
      floorLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      floorLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      floorLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
    climberLaserCan = new LaserCan(Constants.ConfingValues.ClimberLaserCANCANID);
    try {
      climberLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      climberLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      climberLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  /** Moves the Kraken X60 climber forward */
  public Command moveForwardCommand() {
    return runOnce(() -> m_climberMotor.set(Constants.CLIMBERSPEED));
  }

  /** Moves the Kraken X60 climber backward */
  public Command moveBackwardCommand() {
    return runOnce(() -> m_climberMotor.set(-Constants.CLIMBERSPEED));
  }

  /**
   * Pull cage in until we have climbed
   * TODO: make sure the robot is off the floor with sensor
   */
  public Command PullCageInCommand() {
    return new RunCommand(() -> m_climberMotor.set(-Constants.CLIMBERSPEED))
        .until(this::isClimberIn)
        .andThen(new InstantCommand(() -> m_climberMotor.set(0), this));
  }

  /** Release climber until climber is out */
  public Command ClimberOutCommand() {
    return new RunCommand(() -> m_climberMotor.set(Constants.CLIMBERSPEED))
        .until(this::isClimberOut)
        .andThen(new InstantCommand(() -> m_climberMotor.set(0), this));

    /**
     * TODO: test this version
     * return new RunCommand(() -> m_climberMotor.set(Constants.CLIMBERSPEED))
     * .until(this::isClimberOut)
     * .andThen(GrabCageCommand());
     */
  }

  /** Run cage intake until cage is in range */
  public Command GrabCageCommand() {
    return new RunCommand(() -> m_intakeMotor.set(Constants.CLIMBERINTAKESPEED))
        .until(this::isCageInRange)
        .andThen(new InstantCommand(() -> m_intakeMotor.set(0), this));
  }

  /** Is climber all the way out? */
  public boolean isClimberOut() {
    return getClimberPositionDegrees() < 29;
  }

  /** Climber is > 80 degrees. AKA all the way in */
  public boolean isClimberIn() {
    return getClimberPositionDegrees() > 80;
  }

  /** Is the cage in the climber? < 35 mm */
  public boolean isCageInRange() {
    LaserCan.Measurement measurement = climberLaserCan.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return (measurement.distance_mm < 35);
    } else {
      return false;
    }
  }

  public Command stopIntakeCommand() {
    return runOnce(() -> m_intakeMotor.set(0));
  }

  /** Stops the Kraken X60 climber */
  public Command stopCommand() {
    return runOnce(() -> m_climberMotor.set(0));
  }

  public double getClimberPositionDegrees() {
    return climberEncoder.get() * 360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Field/Robot/ClimberAngle", getClimberPositionDegrees());
    LaserCan.Measurement measurement = floorLaserCAN.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      Logger.recordOutput("Field/Robot/Climber/floorMM", measurement.distance_mm);
      Logger.recordOutput("Field/Robot/Climber/floorAmbient", measurement.ambient);

    } else {
      System.out.println("Error with reading Floor sensor.");
    }
    LaserCan.Measurement cageMeasurement = climberLaserCan.getMeasurement();
    if (cageMeasurement != null && cageMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      Logger.recordOutput("Field/Robot/Climber/cageMM", cageMeasurement.distance_mm);
      Logger.recordOutput("Field/Robot/Climber/cageAmbient", cageMeasurement.ambient);
    } else {
      System.out.println("Error with reading Floor sensor.");
    }
  }

}
