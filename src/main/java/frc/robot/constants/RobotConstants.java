package frc.robot.constants;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class RobotConstants {
    public static final String canBus = "Ryan Biggee";

    public static class WristConstants {
        public static final Angle wristOffset = Degrees.of(40);
        public static final Angle encoderOffset = Rotations.of(0.587);
        public static final double gearRatio = 75; // 75:1 gear ratio maybe
        public static final double kS = 0; // just before the motor moves.
        public static final double kV = 0;
        public static final double kG = 0; // output necessary to hold the arm horizontally forward
        public static final double kP = 0; // until the output starts to oscillate around the setpoint
        public static final double kI = 0;
        public static final double kD = 0; // as much as possible without introducing jittering to the response
        public static final LinearVelocity maxVelocity = MetersPerSecond.of(20);
        public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(1);
        public static final Angle pidTolerance = Degrees.of(0.5);
    }
}
