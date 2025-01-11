package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralPlacer extends SubsystemBase{
        private final TalonFX placeMotor;

    public CoralPlacer() {
        placeMotor = new TalonFX(34);
    }

    public Command runAtSpeed(double speed) {
        return run(() -> placeMotor.set(speed)).finallyDo(() -> placeMotor.set(0))
            .withName("Run at speed");
    }
}
