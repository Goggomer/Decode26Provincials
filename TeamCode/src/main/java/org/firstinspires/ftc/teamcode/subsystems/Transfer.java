package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() { }

    private final MotorEx transfer = new MotorEx("transfer");

    public final Command off = new SetPower(transfer, 0).requires(this);
    public final Command on = new SetPower(transfer, 1).requires(this);
    public final Command slow = new SetPower(transfer, 0.8).requires(this);
    public final Command reverse = new SetPower(transfer, -1).requires(this);
}