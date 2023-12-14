// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import java.sql.Time;

import javax.lang.model.util.ElementScanner6;

public class ClimbSub extends SubsystemBase {
  /** Creates a new ClimbSub. */

  private ClimbSub climb;

  private CANSparkMax backClimb;
  private CANSparkMax rightClimb;

  private DoubleSolenoid backBrake;

  //private MotorControllerGroup winchMotors;

  private XboxController driver, gunner;


  public ClimbSub() 
  {
    backClimb = new CANSparkMax(Constants.ClimbConstants.backClimb, MotorType.kBrushless);
    rightClimb = new CANSparkMax(Constants.ClimbConstants.rightClimb, MotorType.kBrushless);

    backBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4,5);


    driver = new XboxController(Constants.Driver);
    gunner = new XboxController(Constants.Gunner);

    // rightClimb.restoreFactoryDefaults();
    // backClimb.restoreFactoryDefaults();

    rightClimb.setIdleMode(IdleMode.kBrake);
    backClimb.setIdleMode(IdleMode.kBrake);

    // rightClimb.burnFlash();
    // backClimb.burnFlash();
  }

  public void moveClimb(XboxController driver, XboxController gunner)
  {
    if(driver.getRawButtonPressed(6))
    {
      rightClimb.set(1.0);
      backClimb.set(-0.7);
      backBrake.set(kReverse);

      Timer.delay(1.5);

      backClimb.set(-0.7);
      backBrake.set(kReverse);
      rightClimb.stopMotor();

      Timer.delay(4.4);

      rightClimb.set(-0.5);

      Timer.delay(0.4);

      rightClimb.set(-0.5);
      backClimb.stopMotor();
      backBrake.set(kReverse);

      Timer.delay(2.1);
      
      rightClimb.stopMotor();
    }
    else if(driver.getRawButton(4))
    {
      backClimb.set(1.0);
      backBrake.set(kReverse);
    }
    else if(driver.getRawButton(3))
    {
      backClimb.set(-1.0);
      backBrake.set(kReverse);
    }
    else if(driver.getRawButton(2))
    {
      rightClimb.set(0.2);
    }
    else if(driver.getRawButton(1))
    {
      rightClimb.set(-0.2);
    }
    else    
    {
      backClimb.stopMotor();
      backBrake.set(kForward);
      rightClimb.stopMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
