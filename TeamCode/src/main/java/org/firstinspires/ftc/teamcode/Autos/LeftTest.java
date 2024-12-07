package org.firstinspires.ftc.teamcode.Autos;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subArm;
import org.firstinspires.ftc.teamcode.SubSystems.subAutoGrab;

import java.util.Vector;

@Config
@Autonomous(name = "3 on Left")
public class LeftTest extends LinearOpMode {

    // Shoulder motor initializations
    DcMotorEx shoulder;
    DcMotorEx lSlides;
    DcMotorEx rSlides;

    // Grabber initializations
    public CRServo Lroller;
    public CRServo Rroller;
    public Servo rotator;

    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    public class AutoArm {

        subAutoGrab grab;

        public AutoArm(HardwareMap hardwareMap) {

            // Shoulder motor declarations
            shoulder = hardwareMap.get(DcMotorEx.class, "AR");
            lSlides = hardwareMap.get(DcMotorEx.class, "SL");
            rSlides = hardwareMap.get(DcMotorEx.class, "SR");

            shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            shoulder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shoulder.setDirection(DcMotorEx.Direction.REVERSE);
            lSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lSlides.setDirection(DcMotorEx.Direction.REVERSE);

            // Grabber declarations
            Lroller = hardwareMap.get(CRServo.class, "LG");
            Rroller = hardwareMap.get(CRServo.class, "RG");
            rotator = hardwareMap.get(Servo.class, "RTR");

            grab = new subAutoGrab(hardwareMap);

        }

        private void armToPos(int pitch, int slide) {

            // Setting position
            shoulder.setTargetPosition(pitch);
            lSlides.setTargetPosition(slide);
            rSlides.setTargetPosition(slide);

            // Setting power
            shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1.0);
            lSlides.setPower(1.0);
            rSlides.setPower(1.0);

        }

        private boolean reached(int tol) {
            return Math.abs(lSlides.getCurrentPosition() - lSlides.getTargetPosition()) < tol && Math.abs(shoulder.getCurrentPosition() - shoulder.getTargetPosition()) < tol;
        }

        public class ReadyPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.rotator.setPosition(0.5);
                grab.stop();
                armToPos(3800, 10);
                return !reached(30);
            }
        }
        public Action readyPos() {
            return new ReadyPos();
        }

        public class ReadyPosBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.rotator.setPosition(0.5);
                grab.stop();
                armToPos(3800, 3100);
                return !reached(30);
            }
        }
        public Action readyPosBack() {
            return new ReadyPos();
        }

        public class ArmToBin implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.rotator.setPosition(0.5);
                grab.stop();
                armToPos(3690, 3100);
                return !reached(30);
            }
        }
        public Action armToBin() {
            return new ArmToBin();
        }

        public class SlideIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(3300, 10);
                return !reached(20);
            }
        }
        public Action slideIn() {
            return new SlideIn();
        }

        public class PitchDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.stop();
                armToPos(10, 10);
                return !reached(20);
            }
        }
        public Action pitchDown() {
            return new PitchDown();
        }

        public class SlideOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                armToPos(10, 1200);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action slideOut() {
            return new SlideOut();
        }

        public class LastOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                rotator.setPosition(0.75);
                armToPos(10, 1200);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action lastOut() {
            return new LastOut();
        }

        public class Output implements Action {

            double startTime = getRuntime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.outtake(0.2, 0.5);
                return false;
            }
        }
        public Action output() {
            return new Output();
        }
    }

    //RR
    Pose2d initialPose;
    PinpointDrive drive;

    AutoArm arm;

    Vector2d scorePos = new Vector2d(11, -15);
    double scoreAng = Math.toRadians(50);

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(-6, 0, 0);
        drive = new PinpointDrive(hardwareMap, initialPose);

        arm = new AutoArm(hardwareMap);



        TrajectoryActionBuilder part1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, -8))
                .strafeToLinearHeading(scorePos, scoreAng);

        TrajectoryActionBuilder waiter1 = drive.actionBuilder(initialPose)
                .waitSeconds(1);

        TrajectoryActionBuilder waiter2 = drive.actionBuilder(initialPose)
                .waitSeconds(1);

        TrajectoryActionBuilder waiter3 = drive.actionBuilder(initialPose)
                .waitSeconds(1);

        TrajectoryActionBuilder waiter4 = drive.actionBuilder(initialPose)
                .waitSeconds(1);

        telemetry.addData("Program", "Left");
        telemetry.update();

        arm.armToPos(2420, 10);

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        part1.build(),
                        arm.readyPos()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        arm.readyPosBack(),
                        arm.armToBin(),
                        arm.output(),
                        waiter1.build()
                )
        );

        TrajectoryActionBuilder part2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(9, -20), Math.toRadians(-90));

        Actions.runBlocking(
                new SequentialAction(
                        arm.readyPosBack(),
                        arm.readyPos(),
                        part2.build(),
                        arm.pitchDown(),
                        arm.slideOut()
                )
        );

        TrajectoryActionBuilder part3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(scorePos, scoreAng);

        Actions.runBlocking(
                new SequentialAction(
                        arm.readyPos(),
                        part3.build()
                )
        );

        TrajectoryActionBuilder part4 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(18, -20), Math.toRadians(-95));

        Actions.runBlocking(
                new SequentialAction(
                        arm.armToBin(),
                        arm.output(),
                        waiter2.build(),
                        arm.readyPosBack(),
                        arm.slideIn(),
                        part4.build(),
                        arm.pitchDown(),
                        arm.slideOut()
                )
        );

        TrajectoryActionBuilder part5 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(scorePos, scoreAng);

        Actions.runBlocking(
                new SequentialAction(
                        arm.readyPos(),
                        part5.build()
                )
        );

        PoseStorage.heading = drive.pose.heading.toDouble() + Math.PI/2;


        TrajectoryActionBuilder part6 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(5.5, -20), Math.toRadians(-45));

        Actions.runBlocking(
                new SequentialAction(
                        arm.armToBin(),
                        arm.output(),
                        waiter3.build(),
                        arm.readyPosBack(),
                        arm.readyPos()
                )
        );

        /*

        TrajectoryActionBuilder part7 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(scorePos, scoreAng);

        Actions.runBlocking(
                new SequentialAction(
                        arm.readyPos(),
                        part7.build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        arm.armToBin(),
                        arm.output(),
                        waiter4.build(),
                        arm.slideIn()
                )
        );

        .strafeToLinearHeading(new Vector2d(18, 32.5), Math.toRadians(0))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(18, 31.5), Math.toRadians(135))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(18, 32.5), Math.toRadians(0))
                .waitSeconds(2);
         */
    }
}
