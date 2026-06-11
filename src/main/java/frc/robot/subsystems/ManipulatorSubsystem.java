package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Levels;
import frc.robot.LimelightHelpers;

public class ManipulatorSubsystem extends SubsystemBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private Servo servo;

  private Levels currentLevel = Levels.Home;

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int ledLength = 12;
  // private int knightIndex = 0;
  // private int knightDirection = 1;
  // private int knightDelayCounter = 0;
  // private static final int KNIGHT_DELAY_CYCLES = 3;

  public ManipulatorSubsystem(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    //setup servo for linear actuator
    servo = new Servo(7);
    servo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);

    led = new AddressableLED(8); // PWM port
    ledBuffer = new AddressableLEDBuffer(ledLength);
    led.setLength(ledLength);
    led.setData(ledBuffer);
    led.start();
  }

  /**
   * Updates the mechanism visualization using only the elevator height.
   *
   * @param elevatorHeight The height of the elevator.
   */
  public void updateMechanism(double elevatorHeight) {
    // Log the manipulator pose and visualization
    Logger.recordOutput("Field/Robot/ManipulatorMechanism", getManipulatorPose3d());
  }

  /**
   * Returns the current pose of the manipulator.
   * This now only reflects the elevator's vertical position.
   *
   * @return The Pose3d of the manipulator.
   */
  public Pose3d getManipulatorPose3d() {
    // Conversion factor: inches to meters.
    final double inchesToMeters = 0.0254;

    // Calculate the elevator height in meters.
    double elevatorHeightMeters = (elevatorSubsystem.getTargetHeight() * 2) * inchesToMeters;
    Translation3d elevatorTranslation = new Translation3d(0, 0, elevatorHeightMeters);

    Rotation3d manipulatorRotation = new Rotation3d();

    return new Pose3d(elevatorTranslation, manipulatorRotation);
  }

  /**
   * Sets the elevator to a specified level.
   * Arm commands have been removed.
   *
   * @param level The desired level.
   */
  public void setLevel(Levels level) {
    currentLevel = level;
    switch (level) {
      case L1:
        elevatorSubsystem.setTargetHeight(Constants.L1_HEIGHT);
        break;
      case L2:
        elevatorSubsystem.setTargetHeight(Constants.L2_HEIGHT);
        break;
      case L3:
        elevatorSubsystem.setTargetHeight(Constants.L3_HEIGHT);
        break;
      case L4:
        elevatorSubsystem.setTargetHeight(Constants.L4_HEIGHT);
        break;
      default:
      case Home:
        elevatorSubsystem.setTargetHeight(Constants.ConfingValues.ELEVATOR_MIN_HEIGHT);
        break;
    }
  }

  public Command ExtendServo(){
    return new RunCommand(()-> servo.setAngle(50)); //180 is full extention
  }
  public Command CloseServo(){
    return new RunCommand(()-> servo.setAngle(0));
  }

  public Command SetJustLevel(Levels level) {
    return new ConditionalCommand(
        new InstantCommand(() -> {
          setLevel(level);
        }),
        new InstantCommand(() -> System.out.println("Skipped scoring: No coral + in AUTO")),
        () -> hasInnerCoral() // Only run if not auto or coral is present
    );

  }

  public void resetElevator() {
    elevatorSubsystem.resetEncoder();
  }

  public Command ScoreAtLevelParallelCommand(Levels level) {
    System.out.println("ScoreAtLevelParallelCommand Started");

    Command scoreSequence = new SequentialCommandGroup(
        // Step 1: Intake only if we DON'T already have a coral
        new ParallelCommandGroup(
            new InstantCommand(() -> intakeSubsystem.resetSimulationTimer()),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> intakeSubsystem.intake()), // Start intaking
                    new WaitUntilCommand(this::hasCoral), // Wait until we detect the coral
                    new InstantCommand(() -> intakeSubsystem.stopIntake()) // Stop intake immediately once coral is
                                                                           // detected
                ),
                new InstantCommand(() -> System.out.println("Skipping intake - Coral already present")), // Just log
                                                                                                         // that intake
                                                                                                         // was skipped
                this::hasNoCoral // Condition: If we already have coral, skip intake
            ),
            new RunCommand(() -> setLevel(level), this).until(this::isAtHeight) // Move to target level
        ),

        // Ensure the elevator is at height before proceeding (even if intake was
        // skipped)
        new WaitUntilCommand(this::isAtHeight),
        new InstantCommand(() -> intakeSubsystem.resetSimulationTimer()),
        // Step 3: Only score if we have a coral
        new ConditionalCommand(
            new SequentialCommandGroup(
                // new WaitCommand(0.1), // Allow brief stabilization before scoring
                new InstantCommand(() -> intakeSubsystem.release(level)), // Release the coral
                new WaitUntilCommand(() -> !hasCoral())// , // Wait until coral is fully released
            // new WaitCommand(0.025) // Short pause before moving home
            ),
            new InstantCommand(() -> System.out.println("Skipping scoring - No coral detected")), // Just log that we
                                                                                                  // are skipping
                                                                                                  // scoring
            this::hasCoral // Only run this sequence if we actually have a coral
        ),

        // Step 5: Move home
        new InstantCommand(() -> setLevel(Levels.Home), this).until(this::isAtHeight)

    ).finallyDo((interrupted) -> {
      System.out.println("ScoreAtLevelCommand Ended. Stopping Intake.");
      intakeSubsystem.stopIntake();
    });

    // ✅ Only skip if we're in auto AND there's no coral
    return new ConditionalCommand(
        scoreSequence,
        new InstantCommand(() -> System.out.println("Skipped scoring: No coral + in AUTO")),
        () -> !DriverStation.isAutonomous() || hasInnerCoral() // Only run if not auto or coral is present
    );
  }

  public Command releaseCommand() {
    return new InstantCommand(() -> intakeSubsystem.release(currentLevel));
  }

  public Command intakeCommand() {
    return intakeSubsystem.intakeCommandUntilCoral();
  }

  public Command intakeStopCommand() {
    return new InstantCommand(() -> intakeSubsystem.stopIntake());
  }

  public boolean isAtHeight() {
    return elevatorSubsystem.isAtHeight();
  }

  public boolean hasCoral() {
    return intakeSubsystem.isOuterSensorTriggered();
  }

  public boolean hasInnerCoral() {
    return intakeSubsystem.isInnerSensorTriggered();
  }

  public boolean hasNoCoral() {
    return !intakeSubsystem.isOuterSensorTriggered();
  }

  @Override
  public void periodic() {
    double elevatorHeight = elevatorSubsystem.getTargetHeight();
    updateMechanism(elevatorHeight);
    //Robot disabled
    if (!DriverStation.isEnabled()) {
      boolean targetVisible = LimelightHelpers.getTV("limelight-main");
      if (targetVisible) {
        //Green if visible april tag
        for (int i = 0; i < ledLength; i++) {
          ledBuffer.setRGB(i, 0, 255, 0);
        }
      } else {
        for (int i = 0; i < ledLength; i++) {
          ledBuffer.setRGB(i, 255, 0, 0);
        }
      }

    } else {
      //Enabled shows white light
      for (int i = 0; i < ledLength; i++) {
        ledBuffer.setRGB(i, 255, 255, 255);
      }
    }
    led.setData(ledBuffer);
  }
}