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
    public static final class ShooterPorts {
        public static final int LeftFlywheelPort = 16;
        public static final int RightFlywheelPort = 17;
        public static final int pivotPort = 14;
        public static final int rollerPort = 15;
        }
        
        public static final class USBOrder {
        public static final int Zero = 0;
        public static final int One = 1;
        }
        public static final class leftFlywheelPIDConsts {
            public static double pidP = 0.0;
            public static double pidI = 0.0;
            public static double pidD = 0.0;
            }
            
            public static final class rightFlywheelPIDConsts {
            public static double pidP = 0.0;
            public static double pidI = 0.0;
            public static double pidD = 0.0;
            }
            public static final class leftFlywheelFF {
                public static final double kS = 0.0;
                public static final double kV = 0.0;
                public static final double kA = 0.0;
                }
                
                public static final class rightFlywheelFF {
                public static final double kS = 0.0;
                public static final double kV = 0.0;
                public static final double kA = 0.0;
                }
                
}
