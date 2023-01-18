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
public final class Constants {
    public static final int MOTOR_NUMBER = 50;
    public static final int joystickport = 0;
    // public static final double goal = 45;
    public static final int driveleftXport = 0;
    public static final int driveleftYport = 1;
    public static final int motorport = 38;
    public static final int buttonport = 1;
    public static final double zero_position_value = 2000;  // the Unit is unit: 4096 units = 360 degrees 

    public static final double goal = 0;

    public static final int openmv_port = 0; //9

    // public static final int intake_port=0; //1
    // public static final int arm_port=0; //6
}
