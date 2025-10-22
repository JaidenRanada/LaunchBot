/*
   Program Purpose:

   Version of Robot with complete functionality added, however with complete manual control
   over output RPM.

 */

package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="betav1")
public class betav1 extends OpMode {

    CRServo intakeLeft = null;
    CRServo intakeRight = null;

    CRServo lowerLeftChamber = null;
    CRServo lowerRightChamber = null;
    CRServo upperLeftChamber = null;
    CRServo upperRightChamber = null;

    Servo gate = null;

    DcMotor leftFront = null;
    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor rightBack = null;
    DcMotorEx leftFlyWheel = null;
    DcMotorEx rightFlyWheel = null;

    @Override
    public void init() {

        // Chamber Servos
        lowerLeftChamber = hardwareMap.get(CRServo.class, "backLeftS");
        lowerRightChamber = hardwareMap.get(CRServo.class, "backRightS");
        upperLeftChamber = hardwareMap.get(CRServo.class, "frontLeftS");
        upperRightChamber = hardwareMap.get(CRServo.class, "frontRightS");
        lowerRightChamber.setDirection(DcMotorSimple.Direction.FORWARD);
        lowerLeftChamber.setDirection(DcMotorSimple.Direction.FORWARD);
        upperRightChamber.setDirection(DcMotorSimple.Direction.FORWARD);
        upperLeftChamber.setDirection(DcMotorSimple.Direction.FORWARD);

        // Intake
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        intakeRight.setDirection(CRServo.Direction.FORWARD);

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

    }

    // Constants

    boolean a_pressed_previous = false;
    double targetVelocity;
    double flyWheelRPM = 4000;
    double flywheelTPR = 28;

    @Override
    public void loop() {

        wheelMath();

        targetVelocity = (flyWheelRPM / 60) * flywheelTPR;
        leftFlyWheel.setVelocity(targetVelocity);
        rightFlyWheel.setVelocity(targetVelocity);

        intakeLeft.setPower(1);
        intakeRight.setPower(1);

        if (gamepad1.a && !a_pressed_previous) {
            if(gate.getPosition() == 1) {
                gate.setPosition(0);
            } else if (gate.getPosition() == 0) {
                gate.setPosition(1);
            }
        }
        a_pressed_previous = gamepad1.a;

        telemetry.addData("Target RPM", flyWheelRPM);

    }

    // Methods
    public void wheelMath() {

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

    public void TrainingData() {

        InterpLUT lut = new InterpLUT();



    }

}