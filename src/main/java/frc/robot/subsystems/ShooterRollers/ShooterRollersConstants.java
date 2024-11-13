package frc.robot.subsystems.ShooterRollers;

import frc.robot.Constants;

public class ShooterRollersConstants {

    double Tolerance = 10.00;
    public static final Gains gains =
        switch (Constants.currentMode) {
            case SIM -> new Gains(0, 0, 0, 0, 0, 0);
            case REAL -> new Gains(1, 0, 0, 0, 0.13, 0);
            default -> new Gains(1, 0, 0, 0, 0.13, 0);
        };

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
