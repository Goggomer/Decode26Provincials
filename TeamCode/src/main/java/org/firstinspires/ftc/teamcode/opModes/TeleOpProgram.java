package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.helpers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class TeleOpProgram extends NextFTCOpMode {
    public TeleOpProgram() {
        addComponents(
                drive = new FieldCentricDrive(),
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    FieldCentricDrive drive;
    GoBildaPinpointDriver odo;
    Follower follower;

    @Override
    public void onStartButtonPressed() {

        Gamepads.gamepad1().a()
                .whenBecomesTrue(Robot.INSTANCE.ShootFar);

        Gamepads.gamepad1().b()
                .whenBecomesTrue(
                        Robot.INSTANCE.IntakeOn
                );

        Gamepads.gamepad1().x()
                .whenBecomesTrue(
                        Robot.INSTANCE.RobotOff
                );

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(
                        Robot.INSTANCE.TurretClose
                );

        Gamepads.gamepad1().options().whenTrue(() -> drive.getOdo().resetPosAndIMU());
    }
    @Override
    public void onUpdate() {
        BindingManager.update();
        drive.update();
    }
}