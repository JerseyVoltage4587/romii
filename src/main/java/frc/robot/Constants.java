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
 * 
 * kv - motor voltage to cruise at a specific velocity (in meters/sec)
 * ka - motor voltage to accelerate at a specific rate (in meters/sec ^2)
 * ks - motor voltage to accelerate at a certain rate (meters/sec)
 * 
 */

public final class Constants {
    public static final double ksVoltsLeft = 0.4; //0.929
    public static final double kvVoltsLeft = 7.5; //6.33
    public static final double kaVoltsLeft = 0.15; //0.0389
    public static final double ksVoltsRight = 0.4; //0.929
    public static final double kvVoltsRight = 7.8; //6.33
    public static final double kaVoltsRight = 0.15; //0.0389
    public static final double kpDriveVelLeft = 2; //0.0385
    public static final double kpDriveVelRight = 2.5; //0.0385
    public static final double kMaxSpeed = 0.6; //0.8
    public static final double kMaxAcceleration = 0.8;
    public static final double kSecondsPerCycle = 0.020;
    
}
