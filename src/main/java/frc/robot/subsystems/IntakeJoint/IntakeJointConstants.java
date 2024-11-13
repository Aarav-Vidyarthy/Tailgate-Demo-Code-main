package frc.robot.subsystems.IntakeJoint;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeJointConstants {
    public static final double tolerance = Units.degreesToRotations(5);
    public static final double homingCurrent = .5;

    public static final Gains gains =
        switch (Constants.currentMode) {
            case SIM -> new Gains(0, 0, 0, 0, 0, 0);
            case REAL -> new Gains(0, 0, 0, 0, 0, 0);
            default -> new Gains(0, 0, 0, 0, 0, 0);
        };

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
