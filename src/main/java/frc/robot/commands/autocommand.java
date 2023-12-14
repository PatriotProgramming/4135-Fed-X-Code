// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import javax.tools.ForwardingFileObject;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;


public class autocommand extends CommandBase {
  private DriveTrain driveTrain;
  private Intake intake;
  private Timer timer;
  private double startTime;
  private final double runTime = 5.0;

  public autocommand(DriveTrain driveTrain, Intake intake) {
    addRequirements(driveTrain);
    addRequirements(intake);
    this.driveTrain = driveTrain;
    this.intake = intake;
    timer = new Timer();

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    startTime = timer.get();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
   // System.out.println("Pubis");
    driveTrain.runMotorsAutoDist(120, 0.4, timer.get(), 0.0, 4.0);
    driveTrain.endStop(timer.get(), 4.01);
  // driveTrain.endStop(5.1);
  // turret.runTurret(5.1, 6.0, 0.5);
  // frontShoot.runFrontShoot(5.1, 20.0, 0.5);
  // backShoot.runBackShoot(5.1, 20.0, -0.5);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.runMotors(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > startTime + runTime;
  }
}