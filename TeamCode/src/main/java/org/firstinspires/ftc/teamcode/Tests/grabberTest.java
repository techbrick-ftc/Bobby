package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name="Grabber Test")
public class grabberTest extends LinearOpMode {

    private ColorSensor colorSensor;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue;
    private double targetValue = 1000;
    CRServo leftGrabber;
    CRServo rightGrabber;

    public void getColor()  {
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();

        telemetry.addData("Red", redValue);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo leftGrabber = hardwareMap.get(CRServo.class, "LG");
        CRServo rightGrabber = hardwareMap.get(CRServo.class, "RG");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "CS");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //getColor();

            // setting the grabber servos based on gamepad input
            if (gamepad2.right_bumper) {
                leftGrabber.setPower(1);
                rightGrabber.setPower(-1);
            }
            else if (gamepad2.left_bumper) {
                leftGrabber.setPower(-1);
                rightGrabber.setPower(1);
            }
            else {
                leftGrabber.setPower(0);
                rightGrabber.setPower(0);
            }
        }
    }
}