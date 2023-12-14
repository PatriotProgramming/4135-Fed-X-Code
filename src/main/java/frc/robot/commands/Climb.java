// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSub;

public class Climb extends CommandBase {
  /** Creates a new Climb. */

   private ClimbSub climber;
   private XboxController driver, gunner;
  //  private double speed;
  //  private double stopingPoint;

  public Climb(ClimbSub subsystem, XboxController driver, XboxController gunner) {
    climber = subsystem;
    addRequirements(climber);
    this.driver = driver;
    this.gunner = gunner;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    climber.moveClimb(driver, gunner);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
