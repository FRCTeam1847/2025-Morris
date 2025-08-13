// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX m_motor = new TalonFX(Constants.ConfingValues.ClimberCANID, "rio");

  private final DutyCycleEncoder climberEncoder = new DutyCycleEncoder(8);

  public ClimberSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
        .withPeakReverseTorqueCurrent(Amps.of(-120));
    m_motor.getConfigurator().apply(configs);
    // climberEncoder.setDistancePerRotation(360.0);
    //climberEncoder.setDutyCycleRange(0, 360);
  }

  /** Moves the Kraken X60 climber forward */
  public Command moveForwardCommand() {
    return runOnce(() -> m_motor.set(Constants.CLIMBERSPEED));
  }

  /** Moves the Kraken X60 climber backward */
  public Command moveBackwardCommand() {
    return runOnce(() -> m_motor.set(-Constants.CLIMBERSPEED));
  }

  /** Stops the Kraken X60 climber and enables brake mode. */
  public Command stopCommand() {
    return runOnce(() -> m_motor.set(0));
  }

  public double getClimberPositionDegrees() {
    return climberEncoder.get() * 360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Logger.recordOutput("Field/Robot/ClimberAngle", getClimberPositionDegrees());
  }

}
