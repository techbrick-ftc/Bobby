package org.firstinspires.ftc.teamcode.Autos;

import android.system.StructUtsname;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subShoulder;

@Config
@TeleOp(name = "Turn Test")
@Disabled
public class RDRTurnTest extends LinearOpMode {

    //RR
    Pose2d initialPose;
    PinpointDrive drive;

    int angle = 0;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(0, 0, 0);
        drive = new PinpointDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder turn;

        telemetry.addData("Program", "Turn Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.a) {
                angle = 90;
            }

            else if (gamepad1.b) {
                angle = 180;
            }

            else if (gamepad1.x) {
                angle = 210;
            }

            else if (gamepad1.y) {
                angle = 0;
            }

            if (gamepad1.right_bumper) {

                turn = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(angle));

                Actions.runBlocking(
                        turn.build()
                );
            }

            if (gamepad1.dpad_up) {
                angle += 1;
            }

            if (gamepad1.dpad_down) {
                angle -= 1;
            }

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("h", drive.pose.heading.toDouble());
            telemetry.addData("a", angle);
            telemetry.update();
        }
    }
}
