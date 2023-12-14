// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
  
  import edu.wpi.first.wpilibj.Joystick;
  import edu.wpi.first.wpilibj.XboxController;
  import edu.wpi.first.wpilibj2.command.CommandBase;
  import edu.wpi.first.wpilibj2.command.Subsystem;
  import frc.robot.subsystems.DriveTrain;
  import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.Shooter;
  
  public class LimeLightTracking extends CommandBase {
    /**
     * Creates a new LimeLightTracking.
     */
  /////////////////////////////////////////////////use if statements to set variable to different numbers, defining different instances., ex 1 = bad ball in 2nd slot and good ball in first slot, then if statements that do things depending on the instance
      private Shooter shoot;
      private LimeLightSub limeLight;
      private XboxController driver, gunner;
  
    public LimeLightTracking(LimeLightSub subsystem, Shooter subsystem2, XboxController driver, XboxController gunner) {
      limeLight = subsystem;
      shoot = subsystem2;
      addRequirements(limeLight);
      addRequirements(shoot);
      this.driver = driver;
      this.gunner = gunner;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        limeLight.seekTarget(); //Limelight calculations, not actually doing anything physical

        //shoot.autoShoot(driver, gunner, limeLight.calculatePWR(), limeLight.validTarget(), limeLight.getPowerRight());
        // shoot.autoIndexSoftShoot();
        shoot.turretMove(limeLight.validTarget(), limeLight.getPowerRight()); //move turret left and right

        shoot.testShoot(1.0); //simply set shooter to full power
        shoot.testIndex(); //simple indexing up and down
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }

