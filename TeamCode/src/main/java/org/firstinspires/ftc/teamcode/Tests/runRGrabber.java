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

@TeleOp (name="R grabber")
public class runRGrabber extends LinearOpMode {

    @Override
    public void runOpMode() {

        CRServo rightGrabber = hardwareMap.get(CRServo.class, "RG");
        CRServo leftGrabber = hardwareMap.get(CRServo.class, "LG");

        waitForStart();

        while (opModeIsActive()) {

            rightGrabber.setPower(0.50);
            leftGrabber.setPower(0.50);

            telemetry.addData("Command", gamepad1.left_bumper);
            telemetry.update();
        }
    }
}