// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JacksonInject.Value;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

private CANSparkMax imotor;
private XboxController driver, gunner;

private Compressor compressor;

private DoubleSolenoid  intakeSol1;


public Intake() 
{

  imotor = new CANSparkMax(Constants.IntakeConstants.imotor, MotorType.kBrushless);
  
  // imotor.restoreFactoryDefaults();

  imotor.setIdleMode(IdleMode.kCoast);

  driver = new XboxController(Constants.Driver);
  gunner = new XboxController(Constants.Gunner);

  intakeSol1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
  
}

public void runIntake()
{
  if(driver.getRightTriggerAxis() > 0.5) 
  {
    imotor.set(-1.0);
  } 
  else 
  {
    imotor.stopMotor();
  }

  if(driver.getRawButtonPressed(7))
  {
    intakeSol1.set(kForward);
  }
  else if (driver.getRawButtonPressed(8))
  {
    intakeSol1.set(kReverse);
  }
}

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    
  }
}
