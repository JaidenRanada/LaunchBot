/*
   Program Purpose:

   Version of Robot with complete functionality added, however with complete manual control
   over output RPM.

 */

package org.firstinspires.ftc.teamcode;
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

@TeleOp(name="manaulControlv3")
public class fullManualReOrg extends OpMode {

    double flyWheelTargetVelocity;
    double flyWheelRPM = 4000;
    double flywheelTPR = 28;
    int flyWheelState = 0;
    int chamberState = 0;

    boolean a_pressed_previous = false;
    boolean b_pressed_previous = false;
    boolean x_pressed_previous = false;
    boolean dpad_up_pressed_previous = false;
    boolean dpad_down_pressed_previous = false;

    CRServo lowerLeftChamber = null;
    CRServo lowerRightChamber = null;
    CRServo upperLeftChamber = null;
    CRServo upperRightChamber = null;
    CRServo specialChamber = null;

    CRServo intake = null;

    DcMotor leftFront = null;
    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor rightBack = null;

    DcMotorEx leftFlyWheel = null;
    DcMotorEx rightFlyWheel = null;

    Servo gate = null;

    AprilTagProcessor aprilTag = null;
    VisionPortal visionPortal = null;

    @Override
    public void init() {

        // Chamber Servos
        lowerLeftChamber = hardwareMap.get(CRServo.class, "backLeftS");
        lowerRightChamber = hardwareMap.get(CRServo.class, "backRightS");
        upperLeftChamber = hardwareMap.get(CRServo.class, "frontLeftS");
        upperRightChamber = hardwareMap.get(CRServo.class, "frontRightS");
        specialChamber = hardwareMap.get(CRServo.class, "specialChamber");

        // Intake
        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(CRServo.Direction.FORWARD);

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
                // Optional: Add lens intrinsics for more accurate distance results.
                // .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) //"Webcam 1" must match your config
                .addProcessor(aprilTag)
                .build();

    }

    @Override
    public void loop() {

        double targetTPS = (flyWheelRPM / 60) * flywheelTPR;

        if (flyWheelState == 0) {
            flyWheelTargetVelocity = targetTPS;
        }

        if ((gamepad1.dpad_up && !dpad_up_pressed_previous)) {
            flyWheelRPM += 100;
        }
        dpad_up_pressed_previous = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpad_down_pressed_previous) {
            flyWheelRPM -= 100;
        }
        dpad_down_pressed_previous = gamepad1.dpad_down;

        if (gamepad1.a && !a_pressed_previous) {
            gateLogic();
        }
        a_pressed_previous = gamepad1.a;

        if (gamepad1.b && !b_pressed_previous) {
            flyWheelLogic();
        }
        b_pressed_previous = gamepad1.b;

        if (gamepad1.x && !x_pressed_previous) {
            chamberLogic();
        }
        x_pressed_previous = gamepad1.x;

        wheelLogic();
        leftFlyWheel.setVelocity(flyWheelTargetVelocity);
        rightFlyWheel.setVelocity(flyWheelTargetVelocity);
        intake.setPower(1);

        telemetry.addData("Target RPM", flyWheelRPM);
        telemetry.addData("Actual RPM Left", "%.2f", (leftFlyWheel.getVelocity() * 60) / flywheelTPR);
        telemetry.addData("Actual RPM Right", "%.2f", (rightFlyWheel.getVelocity() * 60) / flywheelTPR);

    }

    // Methods

    public void wheelLogic() {

        double max;

        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

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
        if (chamberState == 1) {
            lowerLeftChamber.setPower(-1);
            lowerRightChamber.setPower(1);
            upperLeftChamber.setPower(-1);
            upperRightChamber.setPower(1);
            chamberState = 0;
        } else if (chamberState == 0) {
            lowerLeftChamber.setPower(0);
            lowerRightChamber.setPower(0);
            upperLeftChamber.setPower(0);
            upperRightChamber.setPower(0);
            chamberState = 1;
        }
    }

    public void flyWheelLogic() {
        if (flyWheelState == 1) {
            flyWheelState = 0;
        } else if (flyWheelState == 0) {
            flyWheelTargetVelocity = 0;
            flyWheelState = 1;
        } else {
            flyWheelState = 0;
        }
    }

    public void gateLogic() {
        if(gate.getPosition() == 1) {
            gate.setPosition(0);
            specialChamber.setPower(-1);
        } else if (gate.getPosition() == 0) {
            gate.setPosition(1);
            specialChamber.setPower(0);
        }
    }

}

