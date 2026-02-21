package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class FlyWheel implements Subsystem {
    public static final FlyWheel INSTANCE = new FlyWheel();
    private FlyWheel() { }

    private final MotorEx shooter = new MotorEx("shooter");

    public final Command off = new SetPower(shooter, 0).requires(this);
    public final Command on = new SetPower(shooter, 1).requires(this);
    public final Command slow = new SetPower(shooter, 0.3).requires(this);
    public final Command reverse = new SetPower(shooter, -0.5).requires(this);
}