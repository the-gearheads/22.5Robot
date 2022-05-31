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
    public static class Drivetrain {
        public static int RIGHT_FRONT_MOTOR = 4;
        public static int RIGHT_REAR_MOTOR = 5;
        public static int LEFT_FRONT_MOTOR = 6;
        public static int LEFT_REAR_MOTOR = 7;

        public static double TRACK_WIDTH = 0.622;

        public static double RIGHT_kV = 2.973787777757068;
        public static double RIGHT_kS = 0.5;

        public static double LEFT_kV = 2.977147245265187;
        public static double LEFT_kS = 0.5;

        public static double LEFT_BACKWARD_kV = 3.248834213489218;
        public static double LEFT_BACKWARD_kS = 0.077369;

        public static double TALON_UNITS_PER_ROTATION = 2048;
        public static double SHAFT_TO_WHEEL_GEAR_RATIO = 12.75;
        public static double WHEEL_CIRCUMFERENCE = 8 * 0.0254 * Math.PI; // 4in wheel?

        public static class Sim {
            // 100% incorrect, need to find out in cad
            public static double JKG_M2 = 2.1;
            // Roughly, need to verify, in kg
            public static double ROBOT_MASS = 75;
        }

    }
}
