// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private CANSparkMax frontRight;
  private CANSparkMax frontLeft;
  private CANSparkMax backRight;
  private CANSparkMax backLeft;

  private XboxController driver;

  private DifferentialDrive robot;

  public Timer time;

  // private LimeLightSub limeLight;

  // public NetworkTable table; //info from limelight
  // public NetworkTableEntry tv;

  

  public DriveTrain() 
  {
    frontRight = new CANSparkMax(Constants.DriveTrainConstants.FrontRight, MotorType.kBrushless);
    frontRight.setInverted(true);
    frontLeft = new CANSparkMax(Constants.DriveTrainConstants.FrontLeft, MotorType.kBrushless);
    backRight = new CANSparkMax(Constants.DriveTrainConstants.BackRight, MotorType.kBrushless);
    backRight.setInverted(true);
    backLeft = new CANSparkMax(Constants.DriveTrainConstants.BackLeft, MotorType.kBrushless);

    driver = new XboxController(Constants.Driver);

    robot = new DifferentialDrive(frontLeft, frontRight);

    // limeLight = new LimeLightSub();

    

    backRight.follow(frontRight);
    backLeft.follow(frontLeft);

    time = new Timer();

    // frontRight.restoreFactoryDefaults();
    // frontLeft.restoreFactoryDefaults();
    // backRight.restoreFactoryDefaults();
    // backLeft.restoreFactoryDefaults();

    frontRight.setIdleMode(IdleMode.kBrake);
    frontLeft.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);

    // frontRight.burnFlash();
    // frontLeft.burnFlash();
    // backRight.burnFlash();
    // backLeft.burnFlash();

    // frontRight.setSmartCurrentLimit(30);
    // frontLeft.setSmartCurrentLimit(30);
    // backRight.setSmartCurrentLimit(30);
    // backLeft.setSmartCurrentLimit(30);
    
    robot.setSafetyEnabled(false);  // NEEDED FOR DIFFERENTIAL DRIVE OUTPUT NOT UPDATED ENOUGH
    
  }

  public void controllerSplitArcade(XboxController driver)
  {  
    if(driver.getLeftTriggerAxis() > 0.5)
    {
      robot.arcadeDrive(-driver.getLeftY() * 0.7, -driver.getRightX() * 0.7);
    }
    else
    {
      robot.stopMotor();
    }

    double distance = frontRight.getEncoder().getPosition();
  }

  public void runMotors(double speed)
  {
      robot.arcadeDrive(speed, 0.0); // drive forward for set speed
    
  }

  public void runMotorsAuto(double speed, double time, double startTime, double endTime)
  {
    if(time >= startTime && time <= endTime)
    {
      robot.arcadeDrive(speed, 0.0); // drive forward for set speed
    }
  }

  public void runMotorsAutoDist(double dist, double speed, double time, double startTime, double endTime)
  {

    //////reset encoders!!!!!!!!!!!!!!!!!!!!!!!!
    

    double avgPos = (frontLeft.getEncoder().getPosition() + frontRight.getEncoder().getPosition()) / 2;
    double inches = avgPos * 18.8495559216; //yea
    System.out.println(inches);
    if(time >= startTime && time <= endTime && inches < dist)
    {
        System.out.println("Pubis");

      robot.arcadeDrive(speed, 0.0); // drive forward for set speed
    }
  }

  public void stopMotors(double time, double startTime, double endTime)
  {
    // double currentTime = time.get();
    if (time >= startTime && time <= endTime) //runs until set time is up (time in seconds)
    {  
      robot.stopMotor(); // stop robot
    }
  }

  public void endStop(double time, double startTime)
  {
    // double currentTime = time.get();
    if (time >= startTime) //runs until set time is up (time in seconds)
    {  
      robot.stopMotor(); // stop robot
    }
  }



  @Override
  public void periodic() { 
  }
}
