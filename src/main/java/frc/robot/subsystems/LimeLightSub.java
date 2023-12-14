// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSub extends SubsystemBase {
  /**
   * Creates a new LemonLime
   */

  public LimeLightSub limelight;

  public NetworkTable table; //info from limelight
  public NetworkTableEntry pipeline, camMode, ledMode, tx, ty, ta, tv;

  public double distance, cameraAngle = 27, cameraHeight = 42 , targetHeight = 103; //values for constant values

  public double horizontalSetpoint = 0, verticalSetpoint;

  private double Kp = 0.00490, Ki = 0.00027, Kd, integral, derivative;

  private double Kpy = 0.0065, Kiy = 0.000, Kdy = 0, integralY, derivativeY;

  private double error, prevError, prevErrory, errory, steeringAdjust, steeringAdjustv;

  private double min,minY, max, leftDriveCalc, rightDriveCalc, kMin = 0.018, kMinY = 0.015;

  public double leftPower, rightPower;

  public double a = -0.0000008662587975, b = 0.0004198236974, c = -0.059921352, d = 3.097712193;  //for quartic equation






  public LimeLightSub() 
  {
    horizontalSetpoint = 0; //reseting
    table = NetworkTableInstance.getDefault().getTable("limelight-daza");
    tv = table.getEntry("tv");
  }


  public double getHorizontalOffset() 
  {
    return tx.getDouble(0.0); //targets horizontal offset from line of robot
  }


  public double getVerticalOffset() 
  {
    return ty.getDouble(0.0); //targets vertical offset from line of robot
  }


  public double getTargetArea() 
  {
    return ta.getDouble(0.0);
  }


  public boolean validTarget() {
    if(tv.getDouble(0.0) == 1.0) 
    {
      return true;
    } 
    else 
    {
      return false; //I think this checks if light sees correct target
    }
  }
  
  public double GetDistance() { //finds dist from robot to target

    final double angle = cameraAngle + getVerticalOffset();
    distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));
    return distance;
  }


  public double calculatePWR(){ //quadratic formula used to find rpm needed
    return ((a*(Math.pow(GetDistance(), 3)) + (b*(Math.pow(GetDistance(), 2)) + (c*(GetDistance())) + d)));
  }


  public void seekTarget() {
    //if(validTarget() == true){
    error = getHorizontalOffset();
    errory = 245 - GetDistance();
    

    if(Math.abs(error) < 0.1) {
      integral = 0;
    }
    else if(Math.abs(error) < 12){
      integral += (error * 0.02);
    }
    

    if(Math.abs(errory) < 1) {
      integral = 0;
    }
    else if(Math.abs(errory) < 40){
      integral += (errory * 0.02);
    }

    if(Math.abs(error) < 1) {
      min = 0;
    }else{
      min = kMin;
    }

    if(Math.abs(errory) < 3) {
      minY = 0;
    }else{
      minY = kMinY;
    }

    // if(Math.abs(errory) < 40) {
    //   integralY += (errory * 0.02);
    // }else{
    //   integralY = 0;
    // }
    
    // if(Math.abs(errory) < 1){
      
    // }
      
    
    
    //integralY += -(errory * 0.02);

    derivative = -(error - prevError) / 0.2;
    derivativeY = (errory - prevError) / 0.2;

    if (getHorizontalOffset() > horizontalSetpoint) {
      steeringAdjust = (Kp * error + Ki * integral) + min;
    }

    else if (getHorizontalOffset() < horizontalSetpoint) {
      steeringAdjust = (Kp * error - Ki * integral) - min;
    }

    steeringAdjustv = (Kpy * errory + Kiy * integralY) + minY;

    rightDriveCalc = steeringAdjust; //+ steeringAdjustv;
    leftDriveCalc = steeringAdjust; //- steeringAdjustv;
// }else{
//   rightDriveCalc = 0;
//   leftDriveCalc = 0;
// }

if (validTarget() == false) {
  rightDriveCalc = 0;
  leftDriveCalc = 0;
} 
    
    
    
    leftPower = leftDriveCalc; //+ -steeringAdjustv;
    rightPower = rightDriveCalc; //+ steeringAdjustv;

    prevError = error;
    prevErrory = errory;

}

  public double getPowerLeft(){ //power needed in left motors to center on target
    return leftDriveCalc;
  }

  public double getPowerRight(){ //power needed in right motors to center on target
    return rightDriveCalc;
  }

  public void getLimeLightSmartDashboard(){
    SmartDashboard.putNumber("HorizontalOffset", getHorizontalOffset());
    SmartDashboard.putNumber("VerticalOffset", getVerticalOffset());
    SmartDashboard.putNumber("TargetArea", getTargetArea());
    SmartDashboard.putBoolean("ValidTarget", validTarget());
    SmartDashboard.putNumber("IntegralX", integral);
    SmartDashboard.putNumber("IntegralY", integralY);
    SmartDashboard.putNumber("distance", GetDistance());
    SmartDashboard.putNumber("PWRCalc", calculatePWR());
  }

  @Override
  public void periodic() {
    // System.out.print("Left:");
    // System.out.printvn(getTankDriveTrackingPowerLeft());
    //System.out.print(validTarget());
    // System.out.printvn(getTankDriveTrackingPowerRight());
    // This method will be called once per scheduler run

    table = NetworkTableInstance.getDefault().getTable("limelight-daza");

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");

    SmartDashboard.putBoolean("Valid Target", validTarget());

    
    //System.out.printvn(getHorizontalOffset());
    //SmartDashboard.putNumber("HorizontalOffset", getHorizontalOffset());
    getLimeLightSmartDashboard();
  }
}
