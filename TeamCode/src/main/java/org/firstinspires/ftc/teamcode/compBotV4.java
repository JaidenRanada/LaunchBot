/*
   Program Purpose:

   Version of Robot with complete functionality added, however with complete manual control
   over output RPM.

 */

package org.firstinspires.ftc.teamcode;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




@TeleOp(name="compBotV4")
public class compBotV4 extends OpMode {

    boolean dpad_up_pressed_previous = false;
    boolean dpad_down_pressed_previous = false;
    boolean a_pressed_previous = false;
    boolean x_pressed_previous = false;

    CRServo lowerLeftChamber = null;
    CRServo lowerRightChamber = null;
    CRServo upperLeftChamber = null;
    CRServo upperRightChamber = null;
    CRServo specialChamber = null;

    DcMotor intake = null;

    DcMotor leftFront = null;
    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor rightBack = null;

    double max;
    double axial;
    double lateral;
    double yaw;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    DcMotorEx leftFlyWheel = null;
    DcMotorEx rightFlyWheel = null;
    double flyWheelDesiredRPM = 2000;
    double flywheelTPR = 28;

    Servo gate = null;

    AprilTagProcessor aprilTag = null;
    VisionPortal visionPortal = null;

    private Follower follower;


    @Override
    public void init() {

        // Chamber Servos
        lowerLeftChamber = hardwareMap.get(CRServo.class, "backLeftS");
        lowerRightChamber = hardwareMap.get(CRServo.class, "backRightS");
        upperLeftChamber = hardwareMap.get(CRServo.class, "frontLeftS");
        upperRightChamber = hardwareMap.get(CRServo.class, "frontRightS");
        specialChamber = hardwareMap.get(CRServo.class, "specialChamber");

        // Intake
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Wheels
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Fly Wheels
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Gate
        gate = hardwareMap.get(Servo.class, "gate");

        // Vision
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,90)); //set your starting pose

    }

    double fallbackRPM = 2000;

    boolean start = true;

    boolean locked = false;

    int lockState = 1;

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        follower.update();

        if (gamepad2.a && !a_pressed_previous) {
            gateLogic();
        }
        a_pressed_previous = gamepad2.a;

        if (!locked) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true);
        }

        if (gamepad1.x && !x_pressed_previous) {
            locked = !locked; // Toggle the locked state
            if (!locked) {
                // If we just UNLOCKED, explicitly give control back to the driver
                follower.startTeleopDrive();
            } else {
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), follower.getPose() ))
                                .setConstantHeadingInterpolation(follower.getHeading())
                                .build());
            }
        }
        x_pressed_previous = gamepad1.x; // Update for the next loop



        flyWheelLogic();
        chamberLogic();
        intake.setPower(-1);

        telemetry.addData("Target RPM", flyWheelDesiredRPM);
        telemetry.addData("Actual RPM Left", "%.2f", (leftFlyWheel.getVelocity() * 60) / flywheelTPR);
        telemetry.addData("Actual RPM Right", "%.2f", (rightFlyWheel.getVelocity() * 60) / flywheelTPR);
        telemetry.update();

    }

    // Methods
    public void wheelLogic() {

        if (gamepad1.dpad_down) {
            axial = -.25;
            lateral = 0;
            yaw = 0;
        } else if (gamepad1.dpad_up) {
            axial = .25;
            lateral = 0;
            yaw = 0;
        } else if (gamepad1.dpad_left) {
            axial = 0;
            lateral = -.25;
            yaw = 0;
        } else if (gamepad1.dpad_right) {
            axial = 0;
            lateral = .25;
            yaw = 0;
        } else {
            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;
        }

        frontLeftPower  = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower   = axial - lateral + yaw;
        backRightPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

    }
    public void chamberLogic() {
        lowerLeftChamber.setPower(-1);
        lowerRightChamber.setPower(1);
        upperLeftChamber.setPower(-1);
        upperRightChamber.setPower(1);
    }
    public void flyWheelLogic() {

        if (gamepad2.left_bumper) {
            flyWheelDesiredRPM = 4000;
        } else if (gamepad2.right_bumper) {
            flyWheelDesiredRPM = 5500;
        } else {
            flyWheelDesiredRPM = fallbackRPM;
        }

        if ((gamepad2.dpad_up && !dpad_up_pressed_previous)) {
            fallbackRPM += 100;
        }
        dpad_up_pressed_previous = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !dpad_down_pressed_previous) {
            fallbackRPM -= 100;
        }
        dpad_down_pressed_previous = gamepad2.dpad_down;

        double flyWheelTargetVelocity = (flyWheelDesiredRPM / 60) * flywheelTPR;

        leftFlyWheel.setVelocity(flyWheelTargetVelocity);
        rightFlyWheel.setVelocity(flyWheelTargetVelocity);

    }
    public void gateLogic() {
        if(gate.getPosition() == 1) {
            gate.setPosition(0);
            specialChamber.setPower(0);
        } else if (gate.getPosition() == 0) {
            gate.setPosition(1);
            specialChamber.setPower(-1);
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)


                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

}

