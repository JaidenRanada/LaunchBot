package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Stack;

@Autonomous(name = "reOrgRed6", group = "Examples")
public class reOrgRed6 extends OpMode {

    double flyWheelTargetVelocity;
    double flyWheelRPM = 1750;
    double flywheelTPR = 28;
    CRServo lowerLeftChamber = null;
    CRServo lowerRightChamber = null;
    CRServo upperLeftChamber = null;
    CRServo upperRightChamber = null;
    CRServo specialChamber = null;

    Servo gate = null;

    DcMotor intake = null;

    DcMotorEx leftFlyWheel = null;
    DcMotorEx rightFlyWheel = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // Poses

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    double refX = 56;
    double endX = 24;

    Pose StartPos = new Pose(refX, 17.75 , Math.toRadians(90));
    Pose ShootPos = new Pose(refX, 87, Math.toRadians(135));

    Pose startPickUp1 = new Pose(refX, 84, Math.toRadians(0));
    Pose startPickUp2 = new Pose(refX, 60, Math.toRadians(0));
    Pose startPickUp3 = new Pose(refX, 36, Math.toRadians(0));

    Pose endPickup1 = new Pose(endX, 84, Math.toRadians(0));
    Pose endPickup2 = new Pose(endX, 60, Math.toRadians(0));
    Pose endPickup3 = new Pose(endX, 36, Math.toRadians(0));


    public void Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(StartPos.mirror(), ShootPos.mirror())
                )
                .setLinearHeadingInterpolation(StartPos.mirror().getHeading(), ShootPos.mirror().getHeading())
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootPos.mirror(), startPickUp1.mirror())
                )
                .setLinearHeadingInterpolation(ShootPos.mirror().getHeading(), startPickUp1.mirror().getHeading())
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPickUp1.mirror(), endPickup1.mirror())
                )
                .setLinearHeadingInterpolation(startPickUp1.mirror().getHeading(), endPickup1.mirror().getHeading())
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(endPickup1.mirror(), ShootPos.mirror())
                )
                .setLinearHeadingInterpolation(endPickup1.mirror().getHeading(), ShootPos.mirror().getHeading())
                .build();
        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(ShootPos.mirror(), startPickUp2.mirror())
                )
                .setLinearHeadingInterpolation(ShootPos.mirror().getHeading(), startPickUp2.mirror().getHeading())
                .build();
        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPickUp2.mirror(), endPickup2.mirror())
                )
                .setLinearHeadingInterpolation(startPickUp2.mirror().getHeading(), endPickup2.mirror().getHeading())
                .build();
        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(endPickup2, ShootPos)
                )
                .setLinearHeadingInterpolation(endPickup2.getHeading(), ShootPos.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case -1:
                gate.setPosition(0);
                lowerLeftChamber.setPower(-1);
                lowerRightChamber.setPower(1);
                upperLeftChamber.setPower(-1);
                upperRightChamber.setPower(1);
                intake.setPower(1);
                setPathState(0);
                break;
            case 0:
                follower.followPath(Path1);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    shoot();
                }
                break;

            case 3:

                if (!follower.isBusy()) {
                    follower.followPath(Path2);
                    setPathState(4);
                }

                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Path3, 0.25, true);
                    setPathState(5);
                }

                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(Path4);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    shoot();
                }
                break;
            case 7:
                /*
                follower.followPath(Path5);
                setPathState(8);
                */
                setPathState(98);
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(Path6,0.25,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(Path7);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    shoot();
                }


            case 98:
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), new Pose(24,72).mirror() ) )
                                .setLinearHeadingInterpolation(follower.getHeading(), 90)
                                .build()
                );
                if (!follower.isBusy()) {
                    setPathState(99);
                }
                break;
            case 99:
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(24,72).mirror(), new Pose(12,72).mirror() ) )
                                .setConstantHeadingInterpolation(90)
                                .build()
                );
                break;
        }

        /*
        if (opmodeTimer.getElapsedTimeSeconds() > 26) {
            setPathState(98);
        }

         */
    }

    public void shoot() {
        specialChamber.setPower(-1);
        gate.setPosition(1);
        if (pathTimer.getElapsedTimeSeconds() > 8) {
            setPathState(pathState + 1);
            specialChamber.setPower(0);
            gate.setPosition(0);
        }
    }



    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    double actualRPM = 0;
    double targetTPS = 0;

    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        targetTPS = (flyWheelRPM / 60) * flywheelTPR;
        leftFlyWheel.setVelocity(targetTPS);
        rightFlyWheel.setVelocity(targetTPS);

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("lefttargetTPS",targetTPS);
        telemetry.addData("leftActualTPS",leftFlyWheel.getVelocity());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StartPos.mirror());
        Paths(follower);

        lowerLeftChamber = hardwareMap.get(CRServo.class, "backLeftS");
        lowerRightChamber = hardwareMap.get(CRServo.class, "backRightS");
        upperLeftChamber = hardwareMap.get(CRServo.class, "frontLeftS");
        upperRightChamber = hardwareMap.get(CRServo.class, "frontRightS");
        specialChamber = hardwareMap.get(CRServo.class, "specialChamber");

        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        gate = hardwareMap.get(Servo.class, "gate");

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(-1);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}