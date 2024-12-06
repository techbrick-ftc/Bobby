package org.firstinspires.ftc.teamcode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subAutoGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subShoulder;

@Config
@Autonomous(name = "3 on Right")
public class Right3SpecTemplate extends LinearOpMode {

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

        public class ArmToBar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                grab.stop();
                armToPos(3390, 700);
                return !reached(10);
            }
        }

        public Action armToBar() {
            return new ArmToBar();
        }

        public class Score implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                armToPos(3190, 800);
                return !reached(20);
            }
        }
        public Action score() {
            return new Score();
        }

        public class ArmIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                armToPos(2800, 10);
                return !reached(20);
            }
        }
        public Action armIn() {
            return new ArmIn();
        }

        public class SlideReady implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                grab.stop();
                armToPos(10, 600);
                return !reached(20);
            }
        }
        public Action slideReady() {
            return new SlideReady();
        }

        public class SlideOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                grab.intake(1);
                armToPos(10, 1200);
                return !(reached(20) || grab.checkObjectIn());
            }
        }
        public Action slideOut() {
            return new SlideOut();
        }

        public class Depo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.5);
                armToPos(10, 1200);
                grab.outtake(1);
                return !reached(20);
            }
        }
        public Action depo() {
            return new Depo();
        }
    }

    //RR
    Pose2d initialPose;
    PinpointDrive drive;

    AutoArm arm;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new PinpointDrive(hardwareMap, initialPose);

        arm = new AutoArm(hardwareMap);

        arm.armToPos(2420, 10);

        TrajectoryActionBuilder part1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(22, 0));

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        arm.armToBar(),
                        part1.build()
                )
        );

        TrajectoryActionBuilder part2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(12, 0), Math.toRadians(-90));

        Actions.runBlocking(
                new SequentialAction(
                        arm.score(),
                        arm.armIn(),
                        part2.build()
                )
        );

        TrajectoryActionBuilder part3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(14, -24), Math.toRadians(-45));

        Actions.runBlocking(
                new ParallelAction(
                        arm.slideReady(),
                        part3.build()
                )
        );

        TrajectoryActionBuilder part4 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(10, -24), Math.toRadians(-135));

        Actions.runBlocking(
                new ParallelAction(
                        arm.slideOut(),
                        arm.slideReady(),
                        part4.build(),
                        arm.depo(),
                        arm.slideReady()
                )
        );

        TrajectoryActionBuilder part5 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(14, -30), Math.toRadians(-45));

        Actions.runBlocking(
                new ParallelAction(
                        part5.build(),
                        arm.slideOut(),
                        arm.slideReady()
                )
        );

    }
}
