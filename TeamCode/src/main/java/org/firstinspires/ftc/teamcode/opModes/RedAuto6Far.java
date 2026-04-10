package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
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

@Autonomous(name = "Red Auto: Far 6 Ball", preselectTeleOp = "TeleOpProvincialsRed")
public class RedAuto6Far extends NextFTCOpMode {
    private Paths paths;
    private boolean aborted = false;

    public RedAuto6Far() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, FlyWheel.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, ServoBlocker.INSTANCE, Robot.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        paths = new Paths(follower());
        follower().setStartingPose(new Pose(88, 8, Math.toRadians(0)));
        Globals.alliance = Globals.Alliance.RED;
    }

    @Override
    public void onUpdate() {
        Globals.pose = follower().getPose();
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new FollowPath(paths.Path1),
                Robot.INSTANCE.ShootFar,
                new Delay(2),
                Robot.INSTANCE.IntakeOn,
                new FollowPath(paths.Path2),
                new FollowPath(paths.Path3),
                Robot.INSTANCE.ShootFar,
                new Delay(2),
                new FollowPath(paths.Path4),
                new FollowPath(paths.Path5),
                Robot.INSTANCE.ShootFar,
                new Delay(2),
                Robot.INSTANCE.IntakeOn,
                new FollowPath(paths.Path6)
        ).schedule();
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(88.000, 8.000),
                                    new Pose(85.000, 23.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.000, 23.000),
                                    new Pose(97.752, 36.332),
                                    new Pose(88.089, 35.986),
                                    new Pose(134.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(134.000, 36.000),
                                    new Pose(85.000, 23.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.000, 23.000),
                                    new Pose(110.033, 7.065),
                                    new Pose(134.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(134.000, 9.000),
                                    new Pose(85.000, 23.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.000, 23.000),
                                    new Pose(120.000, 11.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}