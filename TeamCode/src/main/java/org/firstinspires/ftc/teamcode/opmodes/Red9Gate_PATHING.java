package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red - Nine - GATE", group = "Auto")
public class Red9Gate_PATHING extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // declare poses for robot path points
    private final Pose startPose = new Pose(123, 123, Math.toRadians(35));
    private final Pose scorePose = new Pose(121, 121, Math.toRadians(35));
    private final Pose pickup1Pose = new Pose(94,83, Math.toRadians(0));
    private final Pose collect1Pose = new Pose(128, 83, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(94, 57.5, Math.toRadians(0));
    private final Pose collect2Pose = new Pose(133, 57.5, Math.toRadians(0));
    private final Pose finalScorePose = new Pose(119,119, Math.toRadians(35));
    private final Pose frontGatePose = new Pose(112, 63, Math.toRadians(180));
    private final Pose gatePose = new Pose(130, 63, Math.toRadians(180));


    // initialize paths
    private Path scorePreload;
    private PathChain grabPickup1, collectPickup1, scorePickup1, grabPickup2, collectPickup2, moveBack2, scorePickup2, frontGate, hitGate, leaveLine; // for other autos, add more paths

    public void buildPaths() {
        // score preload path
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        collectPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, collect1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), collect1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(collect1Pose, finalScorePose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), finalScorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(finalScorePose, pickup2Pose))
                .setLinearHeadingInterpolation(finalScorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        collectPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, collect2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), collect2Pose.getHeading())
                .build();

        moveBack2 = follower.pathBuilder()
                .addPath(new BezierLine(collect2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(collect2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, finalScorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), finalScorePose.getHeading())
                .build();

        frontGate = follower.pathBuilder()
                .addPath(new BezierLine(finalScorePose, frontGatePose))
                .setLinearHeadingInterpolation(finalScorePose.getHeading(), frontGatePose.getHeading())
                .build();

        hitGate = follower.pathBuilder()
                .addPath(new BezierLine(frontGatePose, gatePose))
                .setLinearHeadingInterpolation(frontGatePose.getHeading(), gatePose.getHeading())
                .build();

        leaveLine = follower.pathBuilder()
                .addPath(new BezierLine(finalScorePose, collect1Pose))
                .setLinearHeadingInterpolation(finalScorePose.getHeading(), collect1Pose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);

                catapult1.setPower(1);
                catapult2.setPower(1);

                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    catapult1.setPower(-1);
                    catapult2.setPower(-1);

                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    intake.setPower(1);

                    follower.followPath(collectPickup1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    catapult1.setPower(1);
                    catapult2.setPower(1);

                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    intake.setPower(0);

                    catapult1.setPower(-1);
                    catapult2.setPower(-1);

                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(frontGate, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(hitGate, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(grabPickup2, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    intake.setPower(1);

                    follower.followPath(collectPickup2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(moveBack2, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    catapult1.setPower(1);
                    catapult2.setPower(1);

                    follower.followPath(scorePickup2, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    intake.setPower(0);
                    catapult1.setPower(-1);
                    catapult2.setPower(-1);

                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(leaveLine, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    requestOpModeStop();
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // Declare catapult and intake motors
    // Declare end-effector members
    private DcMotor intake = null;
    private DcMotorEx catapult1 = null;
    private DcMotorEx catapult2 = null;

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        catapult1 = hardwareMap.get(DcMotorEx.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotorEx.class, "catapult2");
        catapult1.setDirection(DcMotorSimple.Direction.FORWARD);
        catapult2.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {

    }
}
