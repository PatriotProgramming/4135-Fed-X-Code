// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase 
{
  
  private CANSparkMax frontRightShooter, frontLeftShooter, backShooter;

  private XboxController driver;

  private DifferentialDrive robot;

  private CANSparkMax turret;

  private CANSparkMax elevateBack, elevateFront, elevateBottom;

  //limelight
  private LimeLightSub limeLight;

  public NetworkTable table; //info from limelight
  public NetworkTableEntry tv;


  private double c;

  private Timer time;

  
  /** Creates a new Shooter. */
  public Shooter() 
  {
    
    c = 0.13;

    //Shooter Motor Ports
    frontRightShooter = new CANSparkMax(Constants.ShooterConstants.frontRightShooter, MotorType.kBrushless);
    frontLeftShooter = new CANSparkMax(Constants.ShooterConstants.frontLeftShooter, MotorType.kBrushless);
    backShooter = new CANSparkMax(Constants.ShooterConstants.backShooter, MotorType.kBrushless);

    time = new Timer();
    //set both sparks to factory defaults
    frontRightShooter.restoreFactoryDefaults();
    frontLeftShooter.restoreFactoryDefaults();
    backShooter.restoreFactoryDefaults();

    //set the modes for both sparks
    frontRightShooter.setIdleMode(IdleMode.kCoast);
    frontLeftShooter.setIdleMode(IdleMode.kCoast);
    backShooter.setIdleMode(IdleMode.kCoast);

    //Give a current draw limit to the spark max's
    frontRightShooter.setSmartCurrentLimit(40);
    frontLeftShooter.setSmartCurrentLimit(40);
    backShooter.setSmartCurrentLimit(40);

    // frontRightShooter.enableVoltageCompensation(12);
    // frontLeftShooter.enableVoltageCompensation(12);
    // backShooter.enableVoltageCompensation(12);

    //burn all settings on spark max
    frontRightShooter.burnFlash();
    frontLeftShooter.burnFlash();
    backShooter.burnFlash();

    //xbox port
    driver = new XboxController(Constants.Driver);

    //turret port
    turret = new CANSparkMax(Constants.TurretConstants.turretTurn, MotorType.kBrushless);

    //Indexer Constants
    elevateBack = new CANSparkMax(Constants.IndexerConstants.elevateBack, MotorType.kBrushless);
    elevateBottom = new CANSparkMax(Constants.IndexerConstants.elevateBottom, MotorType.kBrushless);
    elevateFront = new CANSparkMax(Constants.IndexerConstants.elevateFront, MotorType.kBrushless);
    
  }


  




  public void testShoot(double setpoint)
  {

    if(driver.getRawButton(2))
    {
      frontRightShooter.set(setpoint);
      frontLeftShooter.set(-(setpoint));
      backShooter.set(setpoint);
    }
    else
    {
      frontRightShooter.stopMotor();
      frontLeftShooter.stopMotor();
      backShooter.stopMotor();
    }

  }

  public void testIndex()
  {

    
    if(driver.getRawButton(1))
    {
      
      elevateBottom.set(-0.8);
      elevateBack.set(0.8);
      elevateFront.set(-0.8);
    }
    else if(driver.getRawButton(3))
    {
      elevateBottom.set(0.8);
      elevateBack.set(-0.8);
      elevateFront.set(0.8);
    }
    else 
    {
      elevateBottom.stopMotor();
      elevateBack.stopMotor();
      elevateFront.stopMotor();
    }
  }

  public void turretMove(boolean valid, double power)
  {
    SmartDashboard.putNumber("Power", power);

      if(driver.getRawButton(8))
      {
        turret.set(.2);
      }
      else if(driver.getRawButton(7))
      {
        turret.set(-.2);
      }
      else
      {
        turret.stopMotor();
      }
  }

  
  @Override
  public void periodic() 
  {

  }

}