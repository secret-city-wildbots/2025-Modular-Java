
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import Constants
import frc.robot.Constants.OperatorConstants;

// Import Subsystems
import frc.robot.Actors.Subsystems.Intake;
// import frc.robot.Actors.Subsystems.Pivot;
// import frc.robot.Actors.Subsystems.Pivot2;

// Import Commands
// import frc.robot.commands.auto.Autos;
import frc.robot.Commands.Intake.*;
// import frc.robot.commands.pivot.*;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake(0);
//   private final Pivot pivot = new Pivot(2, 3);
//   private final Pivot2 pivot2 = new Pivot2(4, 0);

  // Instantiate a Command Xbox Controller
  private final CommandXboxController driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `IntakeCoralCommand` when the Xbox controller's leftBumper button is pressed, cancel on release
    // Schedule `OuttakeCoralCommand` when the Xbox controller's leftTrigger button is pressed, cancel on release
    driveController.leftBumper().whileTrue(new IntakeCoralCommand(intake));
    driveController.leftTrigger().whileTrue(new OuttakeCoralCommand(intake));

    // Schedule `IntakeAlgaeCommand` when the Xbox controller's rightBumper button is pressed, continues on press
    // Schedule `OuttakeAlgaeCommand` when the Xbox controller's rightTrigger button is pressed, cancel on release, also cancels intake
    driveController.rightBumper().onTrue(new IntakeAlgaeCommand(intake));
    driveController.rightTrigger().whileTrue(new OuttakeAlgaeCommand(intake));
  }
}

