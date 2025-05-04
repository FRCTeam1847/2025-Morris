package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
  private static final double HEIGHT_TOLERANCE = 1;
  private double currentHeight = Constants.ConfingValues.ELEVATOR_MIN_HEIGHT;

  public ElevatorSubsystem() {
    leftMotor = new SparkMax(Constants.ConfingValues.ELEVATORLEFTCANID, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.ConfingValues.ELEVATORRIGHTCANID, MotorType.kBrushless);
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);
    leftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP).i(kI).d(kD).outputRange(-1, 1);
    leftConfig.closedLoopRampRate(0.2); // slow it down maybe?

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

    currentHeight = Math.max(Constants.ConfingValues.ELEVATOR_MIN_HEIGHT,
        Math.min(Constants.ConfingValues.ELEVATOR_MAX_HEIGHT, heightInches)); // Clamp height

    // Convert height to motor rotations
    double requiredRotations = (currentHeight / SPROCKET_CIRCUMFERENCE) * GEAR_RATIO;

    // Set motor position control
    leftClosedLoopController.setReference(requiredRotations, ControlType.kPosition);
  }

  public double getTargetHeight() {
    return currentHeight;
  }

  public double getElevatorHeight() {
    if (RobotBase.isSimulation()) {
      return currentHeight;
    }
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
   // SmartDashboard.putNumber("encoder rotations: ", leftEncoder.getPosition());
  }

  public void resetEncoder() {
    leftEncoder.setPosition(0);
  }

  public void setBrakeMode(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
