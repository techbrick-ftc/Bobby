package org.firstinspires.ftc.teamcode.Autos;

import android.system.StructUtsname;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subShoulder;

@Config
@Autonomous(name = "Blue Right Test")
@Disabled
public class BlueRightTest extends LinearOpMode {

    public class AutoArm {

        subShoulder should;
        subGrab grab;

        public AutoArm(HardwareMap hardwareMap) {

            should = new subShoulder(hardwareMap);
            grab = new subGrab(hardwareMap);

            grab.stop();
            grab.rotator.setPosition(0.5);
        }

        public class ArmToBar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                should.setShld(3290, 1);
                should.setSlides(810, 1);
                return false;
            }
        }
        public Action armToBar() {
            return new ArmToBar();
        }

        public class ArmToBarDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                should.setShld(2800, 1);
                should.setSlides(0, 1);
                return false;
            }
        }
        public Action armToBarDown() {
            return new ArmToBarDown();
        }

        public class ArmToWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                should.setShld(1220, 1);
                should.setSlides(850, 1);
                grab.intake(0.5);
                return grab.checkObjectIn();
            }
        }
        public Action armToWall() {
            return new ArmToWall();
        }

        public class ArmBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                should.setShld(3290, 1);
                should.setSlides(0, 1);
                grab.stop();
                return false;
            }
        }
        public Action armBack() {
            return new ArmBack();
        }
    }

    //RR
    Pose2d initialPose;
    PinpointDrive drive;

    AutoArm arm;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(0, 48, Math.toRadians(0));
        drive = new PinpointDrive(hardwareMap, initialPose);

        AutoArm arm = new AutoArm(hardwareMap);

        TrajectoryActionBuilder toBar = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .splineTo(new Vector2d(36, 56), 0);

        TrajectoryActionBuilder back;

        telemetry.addData("Program", "Blue Right Test");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        arm.armToBar(),
                        toBar.build()
                )
        );

        back = drive.actionBuilder(drive.pose)
                .lineToX(16)
                .strafeTo(new Vector2d(16, 25))
                .turnTo(Math.toRadians(180))
                .waitSeconds(1);
                //.lineToXConstantHeading(50);

        Actions.runBlocking(
                new SequentialAction(
                        arm.armToBarDown(),
                        back.build(),
                        arm.armToWall(),
                        arm.armBack()
                )
        );
    }
}
