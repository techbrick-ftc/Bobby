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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name = "States Right")
public class StatesRight extends LinearOpMode {

    //TODO: Shoulder motor initializations
    DcMotorEx shoulder;
    DcMotorEx lSlides;
    DcMotorEx rSlides;

    //TODO: Tune robot positions here
    // x, y, heading
    double[] firstMove = {-50, 2, Math.toRadians(180)};
    double[] toScore = {-29, 2, Math.toRadians(180)};
    double[] clearPos = {-35, 2, Math.toRadians(180)};
    double[] groundPickPos1 = {-33, -20, Math.toRadians(-62)};
    double[] groundPickPos2 = {-23, -30, Math.toRadians(-90)};
    double[] groundPickPos3 = {-21, -36, Math.toRadians(-90)};
    double[] wallPos = {-54, -28, Math.toRadians(180)};

    //TODO: Tune arm positions here
    // pitch, slides
    int[] ready = {2420, 10};
    int[] barInit = {3170, 160};
    int[] barRaise = {3170, 770};
    int[] gGrab1 = {20, 300};
    int[] gGrab2 = {20, 200};
    int[] gGrab3 = {20, 600};
    int[] stash = {20, 1000};
    int[] wall = {1000, 10};

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
                grab.lightGrab();
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
                grab.grab();
                armToPos(barRaise[0], barRaise[1]);
                grab.setWristRotation(depoAng);
                return !reached(20);
            }
        }
        public Action barScore() {
            return new BarScore();
        }

        public class HomeArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.release();
                armToPos(20, 600);
                return !reached(20);
            }
        }
        public Action homeArm() {
            return new HomeArm();
        }

        public class GroundHome implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(10, 200);
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
                grab.stop();
                grab.setWristRotation(inAng);
                armToPos(gGrab1[0], gGrab1[1]);
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
                armToPos(gGrab1[0], gGrab1[1] + 900);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action grab1out() {
            return new Grab1Out();
        }

        public class Grab2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.stop();
                grab.setWristRotation(upAng);
                armToPos(gGrab2[0], gGrab2[1]);
                return !reached(20);
            }
        }
        public Action grab2() {
            return new Grab2();
        }

        public class Grab2Out implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(inAng);
                armToPos(gGrab2[0], gGrab2[1] + 900);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action grab2out() {
            return new Grab2Out();
        }

        public class Grab3 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.stop();
                grab.setWristRotation(inAng);
                armToPos(gGrab3[0], gGrab3[1]);
                return !reached(20);
            }
        }
        public Action grab3() {
            return new Grab3();
        }

        public class Grab3Out implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(inAng);
                armToPos(gGrab3[0], gGrab3[1] + 600);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action grab3out() {
            return new Grab3Out();
        }

        public class PrepWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.setWristRotation(wallInAng);
                armToPos(wall[0], wall[1]);
                return !reached(20);
            }
        }
        public Action prepWall() {
            return new PrepWall();
        }

        public class Depo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.setWristRotation(upAng);
                grab.outtake(1);
                return false;
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

        initialPose = new Pose2d(-62, -10, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, initialPose);

        arm = new AutoArm(hardwareMap);

        arm.armToPos(ready[0], ready[1]);
        grab.setWristRotation(0.1);
        grab.lightGrab();

        TrajectoryActionBuilder first = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(firstMove[0], firstMove[1]), firstMove[2]);


        telemetry.addData("Program", "States Right");
        telemetry.update();

        //arm.readyPos();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                first.build(),
                                arm.barInit()
                        )
                )
        );

        TrajectoryActionBuilder scorePos = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(toScore[0], toScore[1]), toScore[2]);

        Actions.runBlocking(
                new SequentialAction(
                        scorePos.build(),
                        arm.barScore()
                )
        );

        TrajectoryActionBuilder clearing = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(clearPos[0], clearPos[1]), clearPos[2]);

        /*
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                arm.homeArm(),
                                clearing.build()
                        )
                )
        );

         */

        TrajectoryActionBuilder grab1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(groundPickPos1[0], groundPickPos1[1]), groundPickPos1[2]);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                grab1.build(),
                                arm.grab1()
                        ),
                        arm.grab1out()
                )
        );

        TrajectoryActionBuilder grab1Depo = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(groundPickPos1[0] - 6, groundPickPos1[1]), groundPickPos1[2] - Math.toRadians(80));

        Actions.runBlocking(
                new SequentialAction(
                        grab1Depo.build(),
                        arm.depo()
                )
        );

        TrajectoryActionBuilder waiter = drive.actionBuilder(drive.pose)
                .waitSeconds(1);

        Actions.runBlocking(
                new SequentialAction(
                        waiter.build()
                )
        );

        TrajectoryActionBuilder grab2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(groundPickPos2[0], groundPickPos2[1]), groundPickPos2[2]);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                grab2.build(),
                                arm.grab2()
                        ),
                        arm.grab2out()
                )
        );

        TrajectoryActionBuilder grab2Depo = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(groundPickPos2[0] - 12, groundPickPos2[1]), groundPickPos1[2] - Math.toRadians(90));

        Actions.runBlocking(
                new SequentialAction(
                        grab2Depo.build(),
                        arm.depo()
                )
        );

        waiter = drive.actionBuilder(drive.pose)
                .waitSeconds(1);

        Actions.runBlocking(
                new SequentialAction(
                        waiter.build()
                )
        );

        TrajectoryActionBuilder grab3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(groundPickPos3[0], groundPickPos3[1]), groundPickPos3[2]);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                grab3.build(),
                                arm.grab3()
                        ),
                        arm.grab3out()
                )
        );

        TrajectoryActionBuilder grab3Depo = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(groundPickPos3[0] - 12, groundPickPos3[1]), groundPickPos3[2] - Math.toRadians(90));

        Actions.runBlocking(
                new SequentialAction(
                        grab3Depo.build(),
                        arm.depo()
                )
        );

        waiter = drive.actionBuilder(drive.pose)
                .waitSeconds(1);

        Actions.runBlocking(
                new SequentialAction(
                        waiter.build()
                )
        );

        TrajectoryActionBuilder toWall = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(wallPos[0], wallPos[1]), wallPos[2]);

        Actions.runBlocking(
                new ParallelAction(
                        toWall.build(),
                        arm.prepWall()
                )
        );
    }
}
