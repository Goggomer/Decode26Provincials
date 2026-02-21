package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class FlyNew implements Subsystem {
    public static final FlyNew INSTANCE = new FlyNew();
    private double currentTarget = 0;

    private final MotorEx motor = new MotorEx("shooter");
    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.015, 0, 0)
            .basicFF(0.004, 0, 0.05)
            .build();

    public final Command off = new InstantCommand(() -> setTargetSpeed(0)).named("FlywheelOff");
    public final Command slow = new InstantCommand(() -> setTargetSpeed(1240)).named("FlywheelOn");
    public final Command fast = new InstantCommand(() -> setTargetSpeed(1500)).named("FlywheelOn");

    private void setTargetSpeed(double speed) {
        currentTarget = speed;
        controller.setGoal(new KineticState(0, speed, 0));
    }

    public boolean isAtSpeed() {
        if (currentTarget <= 0) return false;

        double currentVelocity = motor.getVelocity();
        return (currentVelocity >= currentTarget - 30 && currentVelocity <= currentTarget + 30);
    }

    @Override
    public void periodic() {
        double velocity = motor.getVelocity();
        double power = controller.calculate(motor.getState());

        // ACTIVE BRAKING: If we are 50 RPM over the target,
        // cut the power or apply a tiny bit of reverse power.
        if (currentTarget > 0 && velocity > (currentTarget + 50)) {
            power = -0.05; // Apply 5% reverse power to drag the speed back down
        }

        motor.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }
}