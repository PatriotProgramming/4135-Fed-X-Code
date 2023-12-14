// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private CANSparkMax frontRight;
  private CANSparkMax frontLeft;
  private CANSparkMax backRight;
  private CANSparkMax backLeft;

  private XboxController driver, gunner;

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
    gunner = new XboxController(Constants.Gunner);

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
      robot.arcadeDrive(-driver.getLeftY() * 0.8, driver.getRightX() * 0.8);
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

  public void stopMotors(double startTime, double endTime)
  {
    double currentTime = time.get();
    if (currentTime >= startTime && currentTime <= endTime) //runs until set time is up (time in seconds)
    {  
      robot.stopMotor(); // stop robot
    }
  }

  public void endStop(double startTime)
  {
    double currentTime = time.get();
    if (currentTime >= startTime) //runs until set time is up (time in seconds)
    {  
      robot.stopMotor(); // stop robot
    }
  }

  // public void seekTarget(boolean valid, double leftPower, double rightPower)
  // {
  //   System.out.println(valid);
  //   System.out.println(leftPower);
  //   System.out.println(rightPower);
  //   if(valid == true)
  //   {  
  //     frontRight.set(rightPower);
  //     frontLeft.set(leftPower);
  //   }
  //   else
  //   {
  //     frontRight.set(0.15);
  //     frontLeft.set(0.15);
  //   }
  // } 
 

  // public void move(double rotations, double speed)
  // {
  //   frontRight.set(speed);
  //   frontLeft.set(speed);
  //   while(encoder < rotations)
  //   {
  //     wait(4.0);
  //   }
  // }
  

 

  //waitUntil()


  @Override
  public void periodic() { 
  }
}
