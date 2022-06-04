// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static final). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain {
        public static final int RIGHT_FRONT_MOTOR = 4;
        public static final int RIGHT_REAR_MOTOR = 5;
        public static final int LEFT_FRONT_MOTOR = 6;
        public static final int LEFT_REAR_MOTOR = 7;

        public static final double TRACK_WIDTH = 0.622;

        public static final double RIGHT_kV = 2.973787777757068;
        public static final double RIGHT_kS = 0.5;

        public static final double LEFT_kV = 2.977147245265187;
        public static final double LEFT_kS = 0.5;

        public static final double TALON_UNITS_PER_ROTATION = 2048;
        public static final double SHAFT_TO_WHEEL_GEAR_RATIO = 12.75;
        public static final double WHEEL_CIRCUMFERENCE = 8 * 0.0254 * Math.PI; // 4in wheel?

        public static final class Sim {
            // 100% incorrect, need to find out in cad
            public static final double JKG_M2 = 5;
            // in kg
            public static final double ROBOT_MASS = 56;
        }

    }

    public static final class Controllers {
        public static final double DRIVE_DEADBAND = 0.08;
        public static final double ROTATE_DEADBAND = 0.08;

        public static final double DEFAULT_DRIVE_SPEED = 2;
        public static final double DEFAULT_ROT_SPEED = 2.2;
    }
}
