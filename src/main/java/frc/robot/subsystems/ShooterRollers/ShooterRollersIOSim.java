package frc.robot.subsystems.ShooterRollers;

import static frc.robot.subsystems.ShooterRollers.ShooterRollersConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterRollersIOSim implements ShooterRollersIO {
    private final FlywheelSim topSim = 
        new FlywheelSim(DCMotor.getKrakenX60(1), 1.0 / 1.0, 24.0/15.0);
    private final FlywheelSim bottomSim = 
        new FlywheelSim(DCMotor.getKrakenX60(1), 1.0 / 1.0, 24.0/15.0);
    
    private final PIDController topController = 
        new PIDController(gains.kP(), gains.kI(), gains.kD());
    private final PIDController bottomController = 
        new PIDController(gains.kP(), gains.kI(), gains.kD());

        private double topAppliedVolts = 0.0;
        private double bottomAppliedVolts = 0.0;

        private Double topSetpointRpm = null;
        private Double bottomSetpointRpm = null;
        private double topFeedforward = 0.0;
        private double bottomFeedForward = 0.0;

        @Override
        public void updateInputs(ShooterRollersIOInputs inputs) {
            topSim.update(0.02);
            bottomSim.update(0.02);

            
        }
}
