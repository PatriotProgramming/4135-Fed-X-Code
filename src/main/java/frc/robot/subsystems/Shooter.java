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

  private XboxController driver, gunner;

  private DifferentialDrive robot;

  private CANSparkMax turret;

  private CANSparkMax elevateBack, elevateFront, elevateBottom;

  private AnalogInput lowerBeamBreakA;
  private AnalogInput lowerBeamBreakB;
  private AnalogInput upperBeamBreakA;
  private AnalogInput upperBeamBreakB;

  //limelight
  private LimeLightSub limeLight;

  public NetworkTable table; //info from limelight
  public NetworkTableEntry tv;

  //color sensor
  private ColorSensorV3 colorSensor;
  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

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
    gunner = new XboxController(Constants.Gunner);

    //turret port
    turret = new CANSparkMax(Constants.TurretConstants.turretTurn, MotorType.kBrushless);

    //Indexer Constants
    elevateBack = new CANSparkMax(Constants.IndexerConstants.elevateBack, MotorType.kBrushless);
    elevateBottom = new CANSparkMax(Constants.IndexerConstants.elevateBottom, MotorType.kBrushless);
    elevateFront = new CANSparkMax(Constants.IndexerConstants.elevateFront, MotorType.kBrushless);

    //Analog initiation
    lowerBeamBreakA = new AnalogInput(Constants.BeamBreakConstants.LOWER_BEAM_A);
    lowerBeamBreakB = new AnalogInput(Constants.BeamBreakConstants.LOWER_BEAM_B);
    upperBeamBreakA = new AnalogInput(Constants.BeamBreakConstants.UPPER_BEAM_A);
    upperBeamBreakB = new AnalogInput(Constants.BeamBreakConstants.UPPER_BEAM_B);

    //limelight
    limeLight = new LimeLightSub();

    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget); 
    
  }


  

  public void autoIndexSoftShoot()
  {
    //beambreak getting values
    double low = lowerBeamBreakB.getValue();
    double up = upperBeamBreakA.getValue();
    double color = colorSensor.getIR();
    

    //Bad Ball true or False
    boolean badBall = false;


    //Color Sensor
    Color detectedColor = colorSensor.getColor();

    
    //Run the color match algorithm on our detected color
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    //Blue Target
    if(match.color == kBlueTarget)
    {
      badBall = true;
    }

    if(badBall == false) //No bad ball
    {
      if(color > 10) //lower has a ball
      {
        if(up > 100) //upper doesn't
        {
          
          elevateBottom.set(-0.2);
          elevateBack.set(0.2);
          elevateFront.set(-0.2);
        }
      }
      else if(up < 100)
      {
        elevateBottom.stopMotor();
        elevateBack.stopMotor();
        elevateFront.stopMotor();
      }
    }
      
    if(badBall == true && up > 100) //bad ball is in and no ball in upper compartment
    {
        
      turret.set(0.5);

      Timer.delay(0.2);
      
      frontRightShooter.set(0.3);
      frontLeftShooter.set(-0.3);
      backShooter.set(0.3);

      elevateBack.set(1.0);
      elevateBottom.set(-1.0);
      elevateFront.set(-1.0);

      turret.stopMotor();

      Timer.delay(1.0);

      elevateBottom.stopMotor();
      elevateBack.stopMotor();
      elevateFront.stopMotor();

      frontRightShooter.stopMotor();
      frontLeftShooter.stopMotor();
      backShooter.stopMotor();

      turret.set(-0.5);

      Timer.delay(0.2);

      turret.stopMotor();
    }
  }



  public void autoShoot(XboxController driver, XboxController gunner, double setpoint, boolean valid, double power)
  {
    //Beam Break Getting Values
    int low = lowerBeamBreakB.getValue();
    int up = upperBeamBreakA.getValue();
    int color = colorSensor.getIR(); //Ball in > 10 ball out < 10

    //Commands for Increaing PWR when needed
    

    // if(gunner.getRawButtonPressed(5))
    // {
    //   c += 0.05; //allows drivers to adjust shooter
    // }
    // if(gunner.getRawButtonPressed(6))
    // {
    //   c -= 0.05; //allows drivers to adjust shooter
    // }

    SmartDashboard.putNumber("C", setpoint + c);

    //Bad Ball true or False
    boolean badBall = false;


    //Color Sensor
    Color detectedColor = colorSensor.getColor();

    
    //Run the color match algorithm on our detected color
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    //Blue Target
    // if(match.color == kBlueTarget)
    // {
    //   badBall = true;
    // }
    
 
    // AUTO SHOOTINGGGGG    separate from indexing
    // if(gunner.getRawButtonPressed(3))
    // {
    //   if(badBall == false) //No Bad Ball
    //   {
    //     if(up < 100 && color > 10)
    //     {
        
    //       if(valid == true)
    //       {  
    //         turret.set(power);
    //       }

    //       frontRightShooter.set(setpoint + c);
    //       frontLeftShooter.set(-(setpoint + c));
    //       backShooter.set(setpoint + c);

    //         Timer.delay(2.0);

    //       elevateBottom.set(1.0);
    //       elevateBack.set(1.0);
    //       elevateFront.set(1.0);

    //         Timer.delay(2.0);
        
    //       elevateBottom.stopMotor();
    //       elevateBack.stopMotor();
    //       elevateFront.stopMotor();

    //       frontRightShooter.stopMotor();
    //       frontLeftShooter.stopMotor();
    //       backShooter.stopMotor();

    //       turret.stopMotor();
    //     }
    //     else if(up < 100 && color < 10)
    //     {
    //       if(valid == true)
    //       {  
    //         turret.set(power);
    //       }

    //       frontRightShooter.set(setpoint += c);
    //       frontLeftShooter.set(-(setpoint += c));
    //       backShooter.set(setpoint += c);

    //         Timer.delay(3.0);

    //       elevateBottom.set(1.0);
    //       elevateBack.set(1.0);
    //       elevateFront.set(1.0);

    //         Timer.delay(1.0);

    //       elevateBottom.stopMotor();
    //       elevateBack.stopMotor();
    //       elevateFront.stopMotor();

    //       frontRightShooter.stopMotor();
    //       frontLeftShooter.stopMotor();
    //       backShooter.stopMotor();
    //     }
    //   }

    //   if(badBall == true) //Bad Ball in Spot 2
    //   {
    //     if(valid == true)
    //     {
    //       turret.set(power);
    //     }

    //     frontRightShooter.set(setpoint += c);
    //     frontLeftShooter.set(-(setpoint += c));
    //     backShooter.set(setpoint += c);

    //       Timer.delay(2.0);

    //     elevateBottom.set(1.0);
    //     elevateBack.set(1.0);
    //     elevateFront.set(1.0);

    //       Timer.delay(1.0);
        
    //     elevateBottom.stopMotor();
    //     elevateBack.stopMotor();
    //     elevateFront.stopMotor();

    //     turret.set(0.5);
        
    //     frontRightShooter.set(0.3);
    //     frontLeftShooter.set(-0.3);
    //     backShooter.set(0.3);

    //     Timer.delay(3.0);

    //     elevateBack.set(-1.0);
    //     elevateBottom.set(1.0);
    //     elevateFront.set(1.0);

    //     turret.stopMotor();

    //     Timer.delay(3.0);

    //     frontRightShooter.stopMotor();
    //     frontLeftShooter.stopMotor();
    //     backShooter.stopMotor();

    //     turret.set(-0.5);

    //     Timer.delay(3.0);

    //     turret.set(0.0);
    //   }
    // }
    // else
    //   {
    //     frontRightShooter.stopMotor();
    //       frontLeftShooter.stopMotor();
    //       backShooter.stopMotor();
    //   }
  }

  public void testShoot(double setpoint)
  {
    if(gunner.getRawButtonPressed(5))
    {
      c += 0.05; //allows drivers to adjust shooter
    }

    if(gunner.getRawButtonPressed(6))
    {
      c -= 0.05; //allows drivers to adjust shooter
    }

    if(gunner.getRawButton(2))
    {
      frontRightShooter.set(setpoint += c);
      frontLeftShooter.set(-(setpoint += c));
      backShooter.set(setpoint += c);
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
    if(gunner.getRawButton(1))
    {
      elevateBottom.set(-0.8);
      elevateBack.set(0.8);
      elevateFront.set(-0.8);
    }
    else if(gunner.getRawButton(3))
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

      if(gunner.getRawButton(8))
      {
        turret.set(.5);
      }
      else if(gunner.getRawButton(7))
      {
        turret.set(-.5);
      }
      else
      {
        turret.stopMotor();
      }
    // if(gunner.getRawButton(8))
    // {
    //   if(valid == true)
    //   {
    //     turret.set(power);
    //   }
    //   else
    //   {
    //     turret.set(0.0);
    //   }
    // }
    // else
    // {
    //   turret.stopMotor();
    // }
  }

  // public void removeChild(double distance)
  // {
  //   double distance = 500;
  //   kickChild(distance);
  // }
  


  //Stopping Shooter Motors


  // public void stopShooterMotor() //CHANGE WHEN BUTTONS ARE DECIDED
  // {
  //   if(xbox.getRawButton(1) && xbox.getRawButton(4) == false)
  //   {
  //     frontLeftShooter.set(0.0);
  //     frontRightShooter.set(0.0);
  //     backLeftShooter.set(0.0);
  //     backShooter.set(0.0);
  //   }
  // }



  //COLOR SENSOR CODE


  public void colorSensing()
  {
    

    Color detectedColor = colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    SmartDashboard.putBoolean("BadBall", match.color == kBlueTarget);

    SmartDashboard.putNumber("Color Sensor IR", colorSensor.getIR());
  }


  // public void runFrontShoot(double startTime, double endTime, double speed)
  // {
  //   double currentTime = time.get();
  //   if (currentTime >= startTime && currentTime <= endTime) //runs until set time is up (time in seconds)
  //   {  
  //     robot.arcadeDrive(0.0, speed); // drive forward for set speed
  //   }
  // }

  // public void runBackShoot(double startTime, double endTime, double speed)
  // {
  //   double currentTime = time.get();
  //   if (currentTime >= startTime && currentTime <= endTime) //runs until set time is up (time in seconds)
  //   {  
  //     robot.arcadeDrive(0.0, speed); // drive forward for set speed
  //   }
  // }

  // public void stopMotors(double startTime, double endTime)
  // {
  //   double currentTime = time.get();
  //   if (currentTime >= startTime && currentTime <= endTime) //runs until set time is up (time in seconds)
  //   {  
  //     robot.stopMotor(); // stop robot
  //   }
  // }

  // public void endStop(double startTime)
  // {
  //   double currentTime = time.get();
  //   if (currentTime >= startTime) //runs until set time is up (time in seconds)
  //   {  
  //     robot.stopMotor(); // stop robot
  //   }
  // }









  
  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Low Beam Break A", lowerBeamBreakA.getValue());
    SmartDashboard.putNumber("Low Beam Break B", lowerBeamBreakB.getValue());
    SmartDashboard.putNumber("Up Beam Break A", upperBeamBreakA.getValue());
    SmartDashboard.putNumber("Up Beam Break B", upperBeamBreakB.getValue());

    SmartDashboard.putBoolean("Up Beam Break A", upperBeamBreakA.getValue() < 100 == true);
    SmartDashboard.putBoolean("Low Beam Break B", lowerBeamBreakB.getValue() < 300 == true);

    colorSensing();
  }






  //UN-USED CODE



  
  // public void limeLightShooting(XboxController xbox, double setpoint)
  // {
    
  //   if(xbox.getRawButton(40000))
  //   {
  //     double c = 0.0;

  //     if(xbox.getRawButton(400))
  //     {
  //       c += 0.01;
  //     }
  //     if(xbox.getRawButton(500))
  //     {
  //       c -= 0.01;
  //     }

  //     frontRightShooter.set(setpoint += c);
  //     frontLeftShooter.set(-(setpoint += c));
  //     backShooter.set(setpoint += c);
  //     backLeftShooter.set(-(setpoint += c));
      
  //   }
  //   else
  //   { 

  //     stopShooterMotor();

  //   }
  // }

  // public void limeLightSoftShoot(XboxController xbox)
  // {

  //   if(xbox.getRawButton(4000))
  //   {

  //       turret.set(0.5);

  //       frontRightShooter.set(0.3);
  //       frontLeftShooter.set(-0.3);
  //       backShooter.set(0.3);
  //       backLeftShooter.set(-0.3);

  //     wait(3.0);

  //       elevateBack.set(-1.0);
  //       elevateBottom.set(1.0);
  //       elevateFront.set(1.0);

  //     wait(3.0);

  //       frontRightShooter.stopMotor();
  //       frontLeftShooter.stopMotor();
  //       backShooter.stopMotor();
  //       backLeftShooter.stopMotor();

  //       turret.set(-0.5);

  //     wait(3.0);

  //       turret.set(0.0);
  //   }
  // }
}