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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subAutoGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;

@Config
@Autonomous(name = "States Right")
public class StatesRight extends LinearOpMode {

    //TODO: Shoulder motor initializations
    DcMotorEx shoulder;
    DcMotorEx lSlides;
    DcMotorEx rSlides;

    //TODO: Tune robot positions here
    // x, y, heading
    double[] toScore = {-36, -10, Math.toRadians(180)};
    double[] clearPos = {-46, -10, Math.toRadians(180)};
    double[] groundPickPos1 = {-38, -30, Math.toRadians(-60)};
    double[] groundPickPos2 = {-30, -40, Math.toRadians(-80)};
    double[] groundPickPos3 = {-30, -46, Math.toRadians(-90)};
    double[] wallPos = {-63, -36, Math.toRadians(0)};

    //TODO: Tune arm positions here
    // pitch, slides
    int[] ready = {2420, 10};
    int[] barInit = {3170, 160};
    int[] barRaise = {3170, 770};
    int[] gGrab1 = {20, 1000};
    int[] gGrab2 = {20, 1700};
    int[] gGrab3 = {20, 2400};
    int[] stash = {20, 1000};
    int[] wall = {2800, 20};

    //TODO: Grab pows
    double inAng = 0.34;
    double upAng = 0.7;
    double depoAng = 0.44;
    double wallInAng = 0.09;

    subGrab grab;

    public class AutoArm {

        public AutoArm(HardwareMap hardwareMap) {

            //TODO: Shoulder motor declarations
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

            grab = new subGrab(hardwareMap);
        }

        //Drive arm to position
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

        //TODO: No grabber code included here

        public class ReadyPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(ready[0], ready[1]);
                return !reached(20);
            }
        }
        public Action readyPos() {
            return new ReadyPos();
        }

        public class BarInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(barInit[0], barInit[1]);
                grab.setWristRotation(depoAng);
                return !reached(20);
            }
        }
        public Action barInit() {
            return new BarInit();
        }

        public class BarScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(barRaise[0], barRaise[1]);
                grab.setWristRotation(depoAng);
                return !reached(20);
            }
        }
        public Action barScore() {
            return new BarScore();
        }

        public class GroundHome implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(10, 500);
                grab.setWristRotation(upAng);
                return !reached(20);
            }
        }
        public Action groundHome() {
            return new GroundHome();
        }

        public class Grab1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(upAng);
                armToPos(gGrab1[0], gGrab2[1]);
                return !reached(20);
            }
        }
        public Action grab1() {
            return new Grab1();
        }

        public class Grab1Out implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(inAng);
                armToPos(gGrab1[0], gGrab1[1] + 600);
                return !grab.checkObjectIn();
            }
        }
        public Action grab1out() {
            return new Grab1Out();
        }

    }

    //RR
    Pose2d initialPose;
    PinpointDrive drive;

    AutoArm arm;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(-64, -10, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, initialPose);

        arm = new AutoArm(hardwareMap);

        arm.armToPos(ready[0], ready[1]);
        grab.setWristRotation(0.1);

        TrajectoryActionBuilder dropOff = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(toScore[0], toScore[1]));


        telemetry.addData("Program", "States Right");
        telemetry.update();

        //arm.readyPos();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                dropOff.build(),
                                arm.barInit()
                        ),
                        arm.barScore()
                )
        );

        TrajectoryActionBuilder clear = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(clearPos[0], clearPos[1]), clearPos[2]);

        Actions.runBlocking(
                new ParallelAction(
                        arm.groundHome(),
                        clear.build()
                )
        );

        TrajectoryActionBuilder toGet1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(groundPickPos1[0], groundPickPos1[1]), groundPickPos1[2]);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                toGet1.build(),
                                arm.grab1()
                        ),
                        arm.grab1out()
                )
        );
    }
}
