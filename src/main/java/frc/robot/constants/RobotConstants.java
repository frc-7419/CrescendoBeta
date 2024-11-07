package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class RobotConstants {
    public static final String canBus = "Ryan Biggee";

    public static class ShooterConstants {
        public static final Angle wristOffset = Degrees.of(40);
        public static final Angle encoderOffset = Rotations.of(0.587);
    }
}
