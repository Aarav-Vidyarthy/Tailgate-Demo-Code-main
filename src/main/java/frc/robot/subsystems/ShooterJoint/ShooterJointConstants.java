package frc.robot.subsystems.ShooterJoint;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShooterJointConstants {
    
    public static final double tolerance = Units.degreesToRotations(2);

    public static final Gains gains =
        switch (Constants.currentMode) {
            case SIM -> new Gains(0, 0, 0, 0, 0, 0);
            case REAL -> new Gains(1, 0, 0, 0, 0.19, 0);
            default -> new Gains(1, 0, 0, 0, 0.19, 0);
        };

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
