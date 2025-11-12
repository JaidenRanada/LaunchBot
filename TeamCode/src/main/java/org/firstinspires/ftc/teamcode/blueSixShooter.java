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

@Autonomous(name = "BlueSix", group = "Examples")
public class blueSixShooter extends OpMode {

    CRServo lowerLeftChamber = null;
    CRServo lowerRightChamber = null;
    CRServo upperLeftChamber = null;
    CRServo upperRightChamber = null;
    CRServo specialChamber = null;
    CRServo intake = null;

    Servo gate = null;

    DcMotorEx leftFlyWheel = null;
    DcMotorEx rightFlyWheel = null;

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;

    double flyWheelRPM = 1750;
    double flywheelTPR = 28;
    double targetTPS = 0;

    private int pathState;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    public void Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56, 17.75), new Pose(56, 87))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56, 87), new Pose(56, 87))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 87.000), new Pose(18.000, 87.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.000, 87.000), new Pose(56.000, 87.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56,87), new Pose(56,120))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135) , Math.toRadians(90))
                .build();
    }

    public void autonomousPathUpdate(double targetTPS) {

        switch (pathState) {
            case 0:
                gate.setPosition(0);
                lowerLeftChamber.setPower(-1);
                lowerRightChamber.setPower(1);
                upperLeftChamber.setPower(-1);
                upperRightChamber.setPower(1);
                intake.setPower(1);
                    setPathState(1);
                break;
            case 1:
                follower.followPath(Path1,1,true);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (Math.abs(leftFlyWheel.getVelocity() - targetTPS) < 25){
                        specialChamber.setPower(-1);
                    } else {
                        specialChamber.setPower(0);
                    }
                    gate.setPosition(1);
                }
                if (opmodeTimer.getElapsedTimeSeconds() > 10)
                {
                    specialChamber.setPower(1);
                    gate.setPosition(0);
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(Path3,.25,true);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Path4);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    if (Math.abs(leftFlyWheel.getVelocity() - targetTPS) < 25){
                        specialChamber.setPower(-1);
                    } else {
                        specialChamber.setPower(0);
                    }
                    gate.setPosition(1);
                }
                if (opmodeTimer.getElapsedTimeSeconds() > 27)
                {
                    gate.setPosition(0);
                    specialChamber.setPower(1);
                    setPathState(6);
                }
            case 6:
                follower.followPath(Path5);
                setPathState(999);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate(targetTPS);

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

    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 17.75, Math.toRadians(90)));
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

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}