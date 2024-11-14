package frc.robot.subsystems.IntakeJoint;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeJointConstants {
    public static final double tolerance = Units.degreesToRotations(5);
    public static final double homingCurrent = .5;

     public static final class PHConstants{

        public static final int IntakeForwardSolenoid = 0;
        public static final int IntakeReverseSolenoid = 1;
        /*public static final int FixedClimberVerticalSolenoid = 2;
        public static final int FixedClimberAngledSolenoid = 3;
        public static final int ExtendingClimberAngledSolenoid = 4;
        public static final int ExtendingClimberVerticalSolenoid = 5;
        public static final int HoodForwardSolenoid = 6;
        public static final int HoodReverseSolenoid = 7; */  
    } 

    public static final Gains gains =
        switch (Constants.currentMode) {
            case SIM -> new Gains(0, 0, 0, 0, 0, 0);
            case REAL -> new Gains(0, 0, 0, 0, 0, 0);
            default -> new Gains(0, 0, 0, 0, 0, 0);
        };

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
