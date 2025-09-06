
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import Constants
import frc.robot.Constants.*;

// Import Subsystems
import frc.robot.Actors.Subsystems.Intake;
import frc.robot.Actors.Subsystems.Drivetrain;

// Import Commands
// import frc.robot.commands.auto.Autos;
import frc.robot.Commands.Intake.*;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake(0);
  private final Drivetrain drivetrain = new Drivetrain();

  // Instantiate drive and manipulator Xbox Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DriverControllerPort);
  private final CommandXboxController manipulatorController = new CommandXboxController(OperatorConstants.ManipulatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    drivetrain.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () ->
        drivetrain.drive(
          // Multiply by max speed to map the joystick unitless inputs to actual units.
          // This will map the [-1, 1] to [max speed backwards, max speed forwards],
          // converting them to actual units.
          driverController.getLeftY(),
          driverController.getLeftX(),
          driverController.getRightX()
        ), drivetrain
      )
    );
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
    manipulatorController.leftBumper().whileTrue(new IntakeCoralCommand(intake));
    manipulatorController.leftTrigger().whileTrue(new OuttakeCoralCommand(intake));

    // Schedule `IntakeAlgaeCommand` when the Xbox controller's rightBumper button is pressed, continues on press
    // Schedule `OuttakeAlgaeCommand` when the Xbox controller's rightTrigger button is pressed, cancel on release, also cancels intake
    manipulatorController.rightBumper().onTrue(new IntakeAlgaeCommand(intake));
    manipulatorController.rightTrigger().whileTrue(new OuttakeAlgaeCommand(intake));
  }
}

