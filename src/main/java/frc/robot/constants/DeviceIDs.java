package frc.robot.constants;

public final class DeviceIDs {
    public enum CanIds {
        // Arm CAN IDs
        shooterWrist(13),

        // Gyro CAN IDss
        pigeon(16),

        wrist(18),
        leftIntakeMotor(7),
        rightIntakeMotor(8),
        climber(30),

        topShooter(19),
        bottomShooter(20),
        serializerBack(21),
        ;

        public final int id;

        CanIds(int id) {
            this.id = id;
        }
    }

    public enum SensorIds {

        // Beambreak DIO ID
        beamBreak(2),

        // Limit switch DIO ID
        limitSwitch(0),
      
        wristEncoder(5);

        public final int id;

        SensorIds(int id) {
            this.id = id;
        }
    }
}