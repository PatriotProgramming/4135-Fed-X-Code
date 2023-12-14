// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{

    public static final int Driver = 0;
    public static final int Gunner = 1;

    public class DriveTrainConstants
    {

        //Left Motor Ports
        public static final int FrontLeft = 1;
        public static final int BackLeft = 2;
    
        //Right Motor Ports
        public static final int FrontRight = 3; 
        public static final int BackRight = 4; 
   
    }

    
    public class ShooterConstants
    {

        //Shooter Ports
        public static final int frontRightShooter = 7;
        public static final int frontLeftShooter = 5;
        public static final int backShooter = 6;

        //Shooter PID
        // public static final double Kp = 0.00025;//0.00025
        // public static final double Ki = 0.0000005;//0.00000165
        // public static final double Kd = 0.012;//0.13
        // public static final double FF = 0.0000933;
        // public static final double IZone = 100;//100
        // public static final double Min = 0;
        // public static final double Max = 1;

    }


    public class ClimbConstants
    {

        //Climber Motor Ports
        public static final int backClimb = 12;
        public static final int rightClimb = 13;
    
    }


    public class IntakeConstants
    {

        public static final int imotor = 19;
    
    }


    public class IndexerConstants
    {

        //Indexer Motor Ports
        public static final int elevateBottom = 18;
        public static final int elevateBack = 16;
        public static final int elevateFront = 17;
    
    }
    

    public class TurretConstants
    {

        public static final int turretTurn = 15;

    }

    public class BeamBreakConstants
    {
        public static final int LOWER_BEAM_A = 1;
        public static final int LOWER_BEAM_B = 0;
        public static final int UPPER_BEAM_A = 2;
        public static final int UPPER_BEAM_B = 3;
        
    }
}       
 