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

@TeleOp(name="motorTEST")
public class motorTEST extends OpMode {

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

        // Wheels
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    int state = 0;

    @Override
    public void loop() {

        if (gamepad1.a && !a_pressed_previous) {

            switch (state) {
                case 0:
                    rightFront.setPower(.1);
                    leftBack.setPower(0);
                    state = 1;
                    break;
                case 1:
                    rightFront.setPower(0);
                    rightBack.setPower(.1);
                    state = 2;
                    break;
                case 2:
                    rightBack.setPower(0);
                    leftFront.setPower(.1);
                    state = 3;
                    break;

                case 3:
                    leftFront.setPower(0);
                    leftBack.setPower(.1);
                    state = 0;
                    break;

            }

        }
        a_pressed_previous = gamepad1.a;

    }
}

