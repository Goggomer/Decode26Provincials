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

@Autonomous(name = "Red Auto: Close 9 Ball", preselectTeleOp = "TeleOpProvincialsRed")
public class RedAuto9Close extends NextFTCOpMode {
    private Paths paths;
    private boolean aborted = false;

    public RedAuto9Close() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, FlyWheel.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, ServoBlocker.INSTANCE, Robot.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        paths = new Paths(follower());
        follower().setStartingPose(new Pose(121, 121, Math.toRadians(0)));
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
                Robot.INSTANCE.ShootClose
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
                                    new Pose(121.000, 121.000),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.000, 84.000),

                                    new Pose(127.000, 84.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.000, 84.000),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.000, 84.000),
                                    new Pose(97.748, 60.208),
                                    new Pose(79.887, 58.415),
                                    new Pose(127.218, 59.218)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.218, 59.218),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }
    }
}