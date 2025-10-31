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

@Autonomous(name = "threeShooter", group = "Examples")
public class threeShooter extends OpMode {

    double flyWheelTargetVelocity;
    double flyWheelRPM = 2400;
    double flywheelTPR = 28;
    CRServo lowerLeftChamber = null;
    CRServo lowerRightChamber = null;
    CRServo upperLeftChamber = null;
    CRServo upperRightChamber = null;
    CRServo specialChamber = null;

    Servo gate = null;

    CRServo intake = null;

    DcMotorEx leftFlyWheel = null;
    DcMotorEx rightFlyWheel = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;

    public void Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.000, 122.000), new Pose(59.500, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(315))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.500, 84.000), new Pose(59.500, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.500, 84.000), new Pose(15.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(360))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.000, 84.000), new Pose(59.500, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(360))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.500, 84.000), new Pose(59.500, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                lowerLeftChamber.setPower(-1);
                lowerRightChamber.setPower(1);
                upperLeftChamber.setPower(-1);
                upperRightChamber.setPower(1);
                intake.setPower(1);
                setPathState(0);
                break;
            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Path2);
                }
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    gate.setPosition(0);
                }
                pathState = 999;
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/

    boolean timerStarted = false;
    Timer fireTimer = new Timer();
    public void fireThree() {

        gate.setPosition(0);

        if (!timerStarted) {
            fireTimer.resetTimer();
            timerStarted = true;
        }

        if (fireTimer.getElapsedTimeSeconds() > 3) {
            gate.setPosition(1);
            timerStarted = false;
            pathState = pathState+1;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        double targetTPS = (flyWheelRPM / 60) * flywheelTPR;
        leftFlyWheel.setVelocity(targetTPS);
        rightFlyWheel.setVelocity(targetTPS);

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));
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

        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(CRServo.Direction.FORWARD);

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