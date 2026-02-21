package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@Autonomous(name = "Blue Auto: Close 9 Ball", preselectTeleOp = "TeleOpProvincialsBlue")
public class BlueAuto9Close extends NextFTCOpMode {
    private Paths paths;
    private boolean aborted = false;

    public BlueAuto9Close() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, FlyNew.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, ServoBlocker.INSTANCE, Robot.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        paths = new Paths(follower());
        follower().setStartingPose(new Pose(22.5, 120.5, Math.toRadians(180)));
        Globals.alliance = Globals.Alliance.BLUE;
    }

    @Override
    public void onUpdate() {
        Globals.pose = follower().getPose();
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new FollowPath(paths.Path1),
                Robot.INSTANCE.ShootClose,
                new Delay(2),
                Robot.INSTANCE.IntakeOn,
                new FollowPath(paths.Path2),
                new FollowPath(paths.Path3),
                Robot.INSTANCE.ShootClose,
                new Delay(2),
                Robot.INSTANCE.IntakeOn,
                new FollowPath(paths.Path4),
                new FollowPath(paths.Path5),
                Robot.INSTANCE.ShootClose,
                new Delay(3),
                Robot.INSTANCE.TurretClose
        ).schedule();
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.586, 120.586),

                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 84.000),

                                    new Pose(16.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.000, 84.000),

                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(31.675, 59.613),
                                    new Pose(64.087, 58.616),
                                    new Pose(15, 59.984)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.217, 59.984),
                                    new Pose(50.935, 54.212),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }
}