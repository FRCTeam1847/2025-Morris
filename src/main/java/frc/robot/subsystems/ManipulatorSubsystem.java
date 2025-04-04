package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Levels;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ManipulatorSubsystem extends SubsystemBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final LoggedMechanism2d combinedMechanism2d;
  private final LoggedMechanismRoot2d baseRoot;
  private final LoggedMechanismLigament2d elevatorLigament;

  private static final int BASE_X = 50;
  private static final int BASE_Y = 10;

  public ManipulatorSubsystem(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    combinedMechanism2d = new LoggedMechanism2d(100, 100);

    // Create the base root for the mechanism visualization
    baseRoot = combinedMechanism2d.getRoot("Base", BASE_X, BASE_Y);

    // Add an elevator ligament (vertical element) to the visualization
    elevatorLigament = baseRoot.append(new LoggedMechanismLigament2d("Elevator", 0, 0));

    SmartDashboard.putData("Combined Mechanism", combinedMechanism2d);
  }

  /**
   * Updates the mechanism visualization using only the elevator height.
   *
   * @param elevatorHeight The height of the elevator.
   */
  public void updateMechanism(double elevatorHeight) {
    // Update the elevator ligament length in the visualization.
    elevatorLigament.setLength(elevatorHeight);

    // Log the manipulator pose and visualization
    Logger.recordOutput("Field/Robot/ManipulatorMechanism", getManipulatorPose3d());
    Logger.recordOutput("Mechanism2d/ManipulatorMechanism", combinedMechanism2d);
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
    switch (level) {
      case CoralStation:
        elevatorSubsystem.setTargetHeight(1);
        break;
      case L1:
        elevatorSubsystem.setTargetHeight(12);
        break;
      case L2:
        elevatorSubsystem.setTargetHeight(6.25);
        break;
      case L3:
        elevatorSubsystem.setTargetHeight(13.75);
        break;
      case L4:
        elevatorSubsystem.setTargetHeight(27);
        break;
      default:
      case Home:
        elevatorSubsystem.setTargetHeight(0.2);
        break;
    }
  }

  public Command ScoreAtLevelCommand(Levels level) {
    System.out.println("Command Started for level " + level);
    return new SequentialCommandGroup(
        // Step 1: Intake until coral is detected
        new InstantCommand(() -> intakeSubsystem.intake()), // Start intaking
        new WaitUntilCommand(this::hasCoral), // Wait until we detect the coral
        new InstantCommand(() -> intakeSubsystem.stopIntake()), // Stop intake once we have coral

        // Step 2: Move to the scoring level
        // new RunCommand(() -> setLevel(level), this).until(this::isAtHeight),
        new InstantCommand(() -> setLevel(level)), // Set the target level
        new WaitUntilCommand(this::isAtHeight), // Wait until the elevator reaches
        // the target height
        // new WaitCommand(0.15), // Small wait before scoring

        // Step 3: Score the coral
        new InstantCommand(() -> intakeSubsystem.release()), // Release the coral
        // new WaitCommand(0.15), // Small wait before continuing
        new WaitUntilCommand(() -> !hasCoral()), // Wait until the coral is no longer detected
        new InstantCommand(() -> intakeSubsystem.stopIntake()), // Stop intake once we have coral

        // Step 4: Wait before returning home
        // new WaitCommand(0.15), // Adjust delay as needed before returning home
        new InstantCommand(() -> setLevel(Levels.Home)) // Move back to home position
    ).finallyDo((interrupted) -> {
      System.out.println("ScoreAtLevelCommand Ended. Stopping Intake.");
      intakeSubsystem.stopIntake();
    });
  }

  public Command ScoreAtLevelParallelCommand(Levels level) {
    System.out.println("Command Started");

    return new SequentialCommandGroup(
        // Step 1: Intake only if we DON'T already have a coral
        new ParallelCommandGroup(
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

        // Step 3: Only score if we have a coral
        new ConditionalCommand(
            new SequentialCommandGroup(
                //new WaitCommand(0.1), // Allow brief stabilization before scoring
                new InstantCommand(() -> intakeSubsystem.release()), // Release the coral
                new WaitUntilCommand(() -> !hasCoral())//, // Wait until coral is fully released
                // new WaitCommand(0.025) // Short pause before moving home
            ),
            new InstantCommand(() -> System.out.println("Skipping scoring - No coral detected")), // Just log that we
                                                                                                  // are skipping
                                                                                                  // scoring
            this::hasCoral // Only run this sequence if we actually have a coral
        ),

        // Step 5: Move home
        new RunCommand(() -> setLevel(Levels.Home), this).until(this::isAtHeight),

        // Ensure the entire sequence completes before allowing auto to drive away
        new WaitUntilCommand(this::isAtHeight) // Make sure elevator reaches home before continuing auto
    ).finallyDo((interrupted) -> {
      System.out.println("ScoreAtLevelCommand Ended. Stopping Intake.");
      intakeSubsystem.stopIntake();
    });
  }

  public Command continuousIntakeCommand() {
    return new RunCommand(() -> {
      if (!hasCoral()) {
        intakeSubsystem.intake();
      } else {
        intakeSubsystem.stopIntake();
      }
    }, intakeSubsystem);
  }

  public Command releaseCommand() {
    return new InstantCommand(() -> intakeSubsystem.release());
  }

  public Command intakeCommand() {
    return intakeSubsystem.intakeCommand();
  }

  public Command intakeStopCommand() {
    return new InstantCommand(() -> intakeSubsystem.stopIntake());
  }

  public boolean isAtHeight() {
    return elevatorSubsystem.isAtHeight();
  }

  public boolean hasCoral() {
    return intakeSubsystem.isSensorTriggered();
  }

  public boolean hasNoCoral() {
    return !intakeSubsystem.isSensorTriggered();
  }

  @Override
  public void periodic() {
    double elevatorHeight = elevatorSubsystem.getTargetHeight();
    updateMechanism(elevatorHeight);
  }
}