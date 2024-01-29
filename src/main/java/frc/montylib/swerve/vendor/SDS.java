package frc.montylib.swerve.vendor;

import frc.montylib.Chassis.Side;

public class SDS {

    /**A class storing configuration of the SDS Mark 4 Inverted Swerve Module */
    public class MK4i {
        public double PIVOT_GEAR_RATIO = 1 / 21.4285714286;

        public enum Motors {
            NEO_V1,
            NEO_VORTEX,
            FALCON_500,
            KRAKEN_X60
        }
    
        public enum AbsoluteEncoder {
            CAN_CODER,
            THRIFTY_ENCODER
        }
    
        public enum GearRatio {
            L1,
            L2,
            L3
        }

        private static double[] L1_Max_Speeds = {
            12.4,
            13.0,
            12.5,
            14.5
        };

        private static double[] L2_Max_Speeds = {
            15.0,
            15.7,
            15.1,
            17.6
        };

        private static double[] L3_Max_Speeds = {
            16.5,
            17.3,
            16.6,
            19.3
        };

        private static int[] Drive_Motor_Can_IDs = {
            1,
            2,
            3,
            4
        };

        private static int[] Pivot_Motor_Can_IDs = {
            5,
            6,
            7,
            8
        };

        private static int[] Absolute_Encoder_Can_IDs = {
            9, 
            10, 
            11,
            12
        };
 
        public static double getMaxSpeed(Motors motor, GearRatio ratio) {
            switch (ratio) {
                case L1: switch (motor) {
                    case KRAKEN_X60: return L1_Max_Speeds[0];
                    case FALCON_500: return L1_Max_Speeds[1];
                    case NEO_V1: return L1_Max_Speeds[2];
                    case NEO_VORTEX: return L1_Max_Speeds[3];
                    default: return 0.0;
                }
                case L2: switch (motor) {
                    case KRAKEN_X60: return L2_Max_Speeds[0];
                    case FALCON_500: return L2_Max_Speeds[1];
                    case NEO_V1: return L2_Max_Speeds[2];
                    case NEO_VORTEX: return L2_Max_Speeds[3];
                    default: return 0.0;
                }
                case L3: switch (motor) {
                    case KRAKEN_X60: return L3_Max_Speeds[0];
                    case FALCON_500: return L3_Max_Speeds[1];
                    case NEO_V1: return L3_Max_Speeds[2];
                    case NEO_VORTEX: return L3_Max_Speeds[3];
                    default: return 0.0;
                }
                default: return 0.0;
            }
        }

        public static double getDriveGearRatio(GearRatio ratio) {
            switch (ratio) {
                case L1: return 1 / 8.14;
                case L2: return 1 / 6.75;
                case L3: return 1 / 6.12;
                default: return 0.0;
            }
        }

        public enum ModuleLocation {
            LEFT_FRONT,
            RIGHT_FRONT,
            LEFT_BACK,
            RIGHT_BACK
        }

        public static int[] getCanIDs(ModuleLocation position) {
            switch (position) {
                case LEFT_FRONT: return new int[] {
                    Drive_Motor_Can_IDs[0],
                    Pivot_Motor_Can_IDs[0],
                    Absolute_Encoder_Can_IDs[0]
                };
                case RIGHT_FRONT: return new int[] {
                    Drive_Motor_Can_IDs[1],
                    Pivot_Motor_Can_IDs[1],
                    Absolute_Encoder_Can_IDs[1]
                };
                case LEFT_BACK: return new int[] {
                    Drive_Motor_Can_IDs[2],
                    Pivot_Motor_Can_IDs[2],
                    Absolute_Encoder_Can_IDs[2]
                };
                case RIGHT_BACK: return new int[] {
                    Drive_Motor_Can_IDs[3],
                    Pivot_Motor_Can_IDs[3],
                    Absolute_Encoder_Can_IDs[3]
                };
                default: return new int[]{};
            }
        }

        public static boolean getDriveMotorReversed(ModuleLocation position, Side side_reversed) {
            switch (position) {
                case LEFT_FRONT: return side_reversed == Side.LEFT ? true : false;
                case RIGHT_FRONT: return side_reversed == Side.RIGHT ? true : false;
                case LEFT_BACK: return side_reversed == Side.LEFT ? true : false;
                case RIGHT_BACK: return side_reversed == Side.RIGHT ? true : false;
                default: return false;
            }
        }
    }
}
