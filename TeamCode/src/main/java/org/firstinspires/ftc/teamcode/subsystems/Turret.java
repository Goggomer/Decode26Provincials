package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    // Initialize with .zeroed() (sets current as 0) and .breakMode()
    private final MotorEx motor = new MotorEx("turret")
            .brakeMode();

    // Small PID for the "rotate 500" movement
    private final ControlSystem controller = ControlSystem.builder()
            .posPid(0.01, 0, 0)
            .build();

    private Turret() { }

    // Command to rotate to 500 ticks and then immediately reset that position to 0
    public final Command rotateAndReset = new SequentialGroup(
            new RunToPosition(controller, 500).requires(this),
            new InstantCommand(() -> motor.zeroed()) // This sets the new position as 0
    ).named("RotateAndReset");

    @Override
    public void periodic() {
        // If a command is active (like RunToPosition), this applies the power
        motor.setPower(controller.calculate(motor.getState()));
    }
}