// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Climb;
//import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LimeLightTracking;
import frc.robot.commands.SplitArcade;
import frc.robot.commands.autocommand;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  XboxController driver = new XboxController(Constants.Driver);
  XboxController gunner = new XboxController(Constants.Gunner);

  //subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  //private final Indexer indexer = new Indexer();
  private final ClimbSub climbSub = new ClimbSub();
  private final LimeLightSub limeLightSub = new LimeLightSub();
 
  //commands
  private final SplitArcade splitArcade = new SplitArcade(driveTrain, driver, gunner);
  private final IntakeCommand intakeCommand = new IntakeCommand(intake, driver, gunner);
  //private final IndexerCommand indexerCommand = new IndexerCommand(indexer, xbox);
  private final Climb climb = new Climb(climbSub, driver, gunner);
  private final LimeLightTracking limeLightTracking = new LimeLightTracking(limeLightSub, shooter, driver, gunner);
  private final autocommand simpleauto = new autocommand(driveTrain, intake);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() 
  {
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(splitArcade); //right trigger + left and right joysticks
    intake.setDefaultCommand(intakeCommand);   //
    shooter.setDefaultCommand(limeLightTracking);
    //climbSub.setDefaultCommand(climb);
    //limeLightSub.setDefaultCommand(limeLightTracking);
    // turret.setDefaultCommand(turretMove);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    // new JoystickButton(xbox, 6).whenHeld(splitArcade);
    // new JoystickButton(xbox, 1).whenHeld(track);
    // new JoystickButton(xbox, 5).whenHeld(limeLightTracking);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
   return simpleauto;// make sequential command group as a command and call it here?
  }
}
