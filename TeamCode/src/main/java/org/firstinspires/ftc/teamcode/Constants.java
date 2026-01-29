package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;

public class Constants {

        public static class ShooterConstants {

            // PID Constants
            // TODO: Tune for built in PID
            public static double kP_ONE = 0.001;
            public static  double kP_TWO = 0.001;
            public static double kI = 0.0;
            public static double kD = 0.000;
            public static double kF = 0.0;
            public static double kV1 = 0.00049;
            public static double kV2 = 0.00044;

            // Vel Targets (ticks per second)
            public static double AUTO_VEL = 300;
            public static double FAR_SHOOT_VEL = 1060;
            public static double CLOSE_SHOOT_VEL = 300;

            // Limits
            public static final double MAX_TICKS_PER_SEC = 2300;
            public static final double IDLE_POWER = 0.2;
            public static final double MAX_POWER = 0.99;

        }


        public static class SpindexerConstants {

            // PID Constants
            public static double INDEXER_KP = 0.0188;
            public static double INDEXER_KI = 0.0;
            public static final double INDEXER_KD = 0.0;
            public static double INDEXER_KF = 0.01;

            // Position Constants (ticks)
            public static final int TICKS_120 = 96;
            public static final int TICKS_60 = 48;
            public static final int TICKS_360 = 288;
            public static double ERROR_THRESHOLD = 5;

            // Limits
            public static double MAX_POWER = 0.6;

            // Anti-Jam Detection Constants
            public static final double MIN_VEL_TO_START_CHECKING = 5;
            public static final double MIN_ERROR_TO_START_CHECKING = 30;
        }


        public static class CollectorConstants {
            // Power Constants
            public static double INTAKE_POWER = 0.95;
            public static double EXTAKE_POWER = -0.8;
            public static double AUTO_POWER = 0.77;
            public static final double OFF_POWER = 0.0;
        }

        public static class FingerConstants {

            // Servo PWM Range
            public static final int DOWN_PWM = 837;
            public static final int UP_PWM = 1258;

            // Servo Position
            public static final double DOWN_POSITION = 0.07;
            public static final double UP_POSITION = 0.99;

            // Timing Constants (seconds)
            public static final double UP_TIME = 0.3;
            public static final double DOWN_TIME = 0.3;
        }

         public static final class RampConstants {

        // Servo PWM Range
        public static final int DOWN_PWM = 837; // change
        public static final int UP_PWM = 1258; // change

        // Servo Position
        public static final double DOWN_POSITION = 0.07; // change
        public static final double UP_POSITION = 0.99; // change
    }

    public static final class PivotConstants {

        // Servo Position
        public static final int LEFT_DOWN_POS = 0; // change
        public static final int LefT_UP_POS = 1; // change

        public static final int RIGHT_DOWN_POS = 0; // change
        public static final int RIGHT_UP_POS = 1; // change
    }

    public static class DriveConstants {

            public static double ALIGNMENT_KP = 1.1;
            public static double ALIGNMENT_KI = 0.13;
            public static double ALIGNMENT_KD = 0.0;

        }
}
