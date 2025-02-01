package org.firstinspires.ftc.teamcode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subAutoGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subShoulder;

@Config
@Disabled
@Autonomous(name = "Turn Test")
public class TurnTest extends LinearOpMode {

    //RR
    Pose2d initialPose;
    PinpointDrive drive;
    @Override
    public void runOpMode() {

        initialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new PinpointDrive(hardwareMap, initialPose);

        waitForStart();

        TrajectoryActionBuilder testMove = drive.actionBuilder(initialPose)
                .turnTo(Math.PI/2)
                .waitSeconds(2)
                .turnTo(Math.PI)
                .waitSeconds(2)
                .turnTo(-Math.PI/2)
                .waitSeconds(2)
                .turnTo(0)
                .waitSeconds(2)
                .turnTo(Math.toRadians(30))
                .waitSeconds(2);


        Actions.runBlocking(
                testMove.build()
        );

    }
}
