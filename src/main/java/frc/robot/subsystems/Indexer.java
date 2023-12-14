// // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project. 

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// // import com.revrobotics.ColorMatch;
// // import com.revrobotics.ColorMatchResult;
// // import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// // import edu.wpi.first.wpilibj.AnalogInput;
// // import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
// // import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


// public class Indexer extends SubsystemBase {
// //   /** Creates a new Indexer. */

//   private CANSparkMax elevateBottom;
//   private CANSparkMax elevateBack;
//   private CANSparkMax elevateFront;

//   private XboxController xbox1;


// //   private AnalogInput lowerBeamBreakA;
// //   private AnalogInput lowerBeamBreakB;
// //   private AnalogInput upperBeamBreakA;
// //   private AnalogInput upperBeamBreakB;

// //   private ColorSensorV3 colorSensor;
// //   private final ColorMatch colorMacher = new ColorMatch();

// //   private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
// //   private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
// //   private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
// //   private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);


// //   //Shooter
// //   private CANSparkMax turret;

// //   private CANSparkMax frontRightShooter, frontLeftShooter, backRightShooter, backLeftShooter;
  
//   public Indexer() 
//   {
//     elevateBottom = new CANSparkMax(Constants.IndexerConstants.elevateBottom, MotorType.kBrushless);
//     elevateBack = new CANSparkMax(Constants.IndexerConstants.elevateBack, MotorType.kBrushless);
//     elevateFront = new CANSparkMax(Constants.IndexerConstants.elevateFront, MotorType.kBrushless);
    
//     xbox1 = new XboxController(1);

// //     colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

// //     colorMacher.addColorMatch(kBlueTarget);
// //     colorMacher.addColorMatch(kGreenTarget);
// //     colorMacher.addColorMatch(kRedTarget);
// //     colorMacher.addColorMatch(kYellowTarget); 
    
// //     AnalogInput lowerBeamBreakA = new AnalogInput(Constants.BeamBreakConstants.LOWER_BEAM_A);
// //     AnalogInput lowerBeamBreakB = new AnalogInput(Constants.BeamBreakConstants.LOWER_BEAM_B);
// //     AnalogInput upperBeamBreakA = new AnalogInput(Constants.BeamBreakConstants.UPPER_BEAM_A);
// //     AnalogInput upperBeamBreakB = new AnalogInput(Constants.BeamBreakConstants.UPPER_BEAM_B);

// //     //shooter
// //     frontRightShooter = new CANSparkMax(Constants.ShooterConstants.frontRightShooter, MotorType.kBrushless);
// //     frontLeftShooter = new CANSparkMax(Constants.ShooterConstants.frontLeftShooter, MotorType.kBrushless);
// //     backRightShooter = new CANSparkMax(Constants.ShooterConstants.backRightShooter, MotorType.kBrushless);
// //     backLeftShooter = new CANSparkMax(Constants.ShooterConstants.backLeftShooter, MotorType.kBrushless);

// //     xbox = new XboxController(0);

// //     turret = new CANSparkMax(Constants.TurretConstants.turretTurn, MotorType.kBrushless);

//   }

//   public void runIndexer()
//   {
//     if(xbox1.getRawButton(3))
//     {
//       elevateBottom.set(1.0);
//       elevateBack.set(1.0);
//       elevateFront.set(1.0);
//     }
//     else
//     {
//       elevateBottom.stopMotor();
//       elevateBack.stopMotor();
//       elevateFront.stopMotor();
//     }    
//   }

// //   public void colorSensing()
// //   {
// //     Color detectedColor = colorSensor.getColor();

// //     /**
// //      * Run the color match algorithm on our detected color
// //      */
// //     String colorString;
// //     ColorMatchResult match = colorMacher.matchClosestColor(detectedColor);

// //     if (match.color == kBlueTarget) 
// //     {
// //       colorString = "Blue";
// //     } 
// //     else if (match.color == kRedTarget) 
// //     {
// //       colorString = "Red";
// //     } 
// //     else if (match.color == kGreenTarget) 
// //     {
// //       colorString = "Green";
// //     } 
// //     else if (match.color == kYellowTarget) 
// //     {
// //       colorString = "Yellow";
// //     } 
// //     else 
// //     {
// //       colorString = "Unknown";
// //     }
// //   }

// //   public void autoIndex()
// //   {
// //     //beambreak getting values
// //     int low = lowerBeamBreakB.getValue();
// //     int up = upperBeamBreakB.getValue(); //CHANGEEEEEE
    
// //     //Bad Ball true or False
// //     boolean badBall = false;

// //     Color detectedColor = colorSensor.getColor();

// //     /**
// //      * Run the color match algorithm on our detected color
// //      */
// //     String colorString;
// //     ColorMatchResult match = colorMacher.matchClosestColor(detectedColor);

// //     if(match.color == kBlueTarget)  // can be changed if needed
// //     {
// //       badBall = true;
// //     }

// //     if(badBall == false) //No bad ball
// //     {
        
// //       if(low < 100) //lower has a ball
// //       {
// //         if(up > 100) //upper doesn't
// //         {
// //           elevateBottom.set(1.0);
// //           elevateBack.set(1.0);
// //           elevateFront.set(1.0);
// //           wait(4.0);
// //         }
// //         else if(up < 100) // upper does
// //         {
// //           elevateBottom.set(0.0);
// //           elevateBack.set(0.0);
// //           elevateFront.set(0.0);
// //         }
// //       }
// //     }
      
// //     if(badBall == true) //bad ball is in
// //     {
// //       if(up < 100 && low < 100)
// //       {
// //         turret.set(0.5);
// //         frontRightShooter.set(0.3);
// //         frontLeftShooter.set(-0.3);
// //         backRightShooter.set(0.3);
// //         backLeftShooter.set(-0.3);

// //         wait(3.0);

// //         elevateBack.set(-1.0);
// //         elevateBottom.set(1.0);
// //         elevateFront.set(1.0);

// //         wait(3.0);

// //         frontRightShooter.stopMotor();
// //         frontLeftShooter.stopMotor();
// //         backRightShooter.stopMotor();
// //         backLeftShooter.stopMotor();

// //         turret.set(-0.5);

// //         wait(3.0);

// //         turret.set(0.0);
// //       }
// //     }
// //   }

// //   private void wait(double sec)
// //   {
// //     double time = 0;
// //     while(time < sec)
// //     {
// //       time += 1;
// //     }
// //   }

//   @Override 
//   public void periodic() 
//   {
//     // colorSensing();
//   }
// }
