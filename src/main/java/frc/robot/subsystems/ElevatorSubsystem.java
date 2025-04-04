package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final SparkClosedLoopController leftClosedLoopController;

  private static final double GEAR_RATIO = 9.0; // 12:1 gearbox
  private static final double SPROCKET_DIAMETER_INCHES = 1.75; // Change this based on your actual sprocket diameter
  private static final double SPROCKET_CIRCUMFERENCE = SPROCKET_DIAMETER_INCHES * Math.PI; // inches per rev
  private static final double kP = 0.0725;
  private static final double kI = 0.0;
  private static final double kD = 0.01;
  private static final double MAX_HEIGHT = 27;
  private static final double HEIGHT_TOLERANCE = 1;
  private double currentHeight = 0.25;
  private boolean isMAXMotionEnabled;

  public ElevatorSubsystem(boolean smartMotion) {
    leftMotor = new SparkMax(11, MotorType.kBrushless);
    rightMotor = new SparkMax(10, MotorType.kBrushless);
    isMAXMotionEnabled = smartMotion;
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);
    if (!smartMotion) {
      leftConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(kP).i(kI).d(kD).outputRange(-1, 1);
      leftConfig.closedLoopRampRate(0.2); // slow it down maybe?

    } else {
      // MAXMotionConfig maxMotionConfig = new MAXMotionConfig()
      // .maxVelocity(1000000) // Set the maximum velocity (in RPM)
      // .maxAcceleration(200000) // Set the maximum acceleration (in RPM per second)
      // .allowedClosedLoopError(0.05) // Set the allowed closed-loop error (in
      // rotations)
      // .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal); // Choose the
      // position control mode

      // // Apply the MAXMotion configuration to the closed-loop settings
      // leftConfig.closedLoop
      // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // .p(kP).i(kI).d(kD)
      // .apply(maxMotionConfig) // Apply the MAXMotion configuration
      // .outputRange(-1, 1); // Set the output range
    }

    leftConfig.inverted(false);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.follow(11, true);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    leftEncoder = leftMotor.getEncoder();
    leftClosedLoopController = leftMotor.getClosedLoopController();

    leftEncoder.setPosition(0);
  }

  public void setTargetHeight(double heightInches) {

    currentHeight = Math.max(0.5, Math.min(MAX_HEIGHT, heightInches)); // Clamp height

    // Convert height to motor rotations
    double requiredRotations = (currentHeight / SPROCKET_CIRCUMFERENCE) * GEAR_RATIO;

    // Set motor position control
    leftClosedLoopController.setReference(requiredRotations,
        isMAXMotionEnabled ? ControlType.kMAXMotionPositionControl : ControlType.kPosition);
  }

  public double getTargetHeight() {
    return currentHeight;
  }

  public double getElevatorHeight() {
    double motorRotations = leftEncoder.getPosition(); // Motor shaft rotations
    double elevatorRotations = motorRotations / GEAR_RATIO; // Convert to elevator shaft rotations
    return elevatorRotations * SPROCKET_CIRCUMFERENCE; // Convert to inches
  }

  public boolean isAtHeight() {
    return Math.abs(getElevatorHeight() - currentHeight) <= HEIGHT_TOLERANCE;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Height: ", getElevatorHeight());
    SmartDashboard.putBoolean("At Height: ", isAtHeight());
    SmartDashboard.putNumber("encoder rotations: ", leftEncoder.getPosition());
  }

  public void resetEncoder() {
    leftEncoder.setPosition(0);
  }
}
