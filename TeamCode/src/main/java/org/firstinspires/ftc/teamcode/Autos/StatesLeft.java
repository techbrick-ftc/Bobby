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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;

import java.util.Vector;

@Config
@Autonomous(name = "States Left")
public class StatesLeft extends LinearOpMode {

    //TODO: Shoulder motor initializations
    DcMotorEx shoulder;
    DcMotorEx lSlides;
    DcMotorEx rSlides;

    //TODO: Sensor initializations
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    //TODO: Tune robot positions here
    // x, y, heading
    double[] move1 = {-61, 35, Math.toRadians(90)};
    double[] scoringPos = {-50, 53, Math.toRadians(135)};
    double[] pickPos1 = {-30, 34, Math.toRadians(65)};
    double[] pickPos2 = {-22, 42, Math.toRadians(90)};
    double[] pickPos3 = {-22, 48, Math.toRadians(90)};

    //TODO: Turn arm positions here
    // pitch, slides
    int[] ready = {1755, 10};
    int[] score = {2700, 3100};
    int[] grabArm1 = {20, 50};
    int[] grabArm2 = {20, 200};
    int[] grabArm3 = {20, 600};

    //TODO: Grab
    subGrab grab;
    double upAng = 0.56;
    double depoAng = 0.26;
    double inAng = 0.34;
    double inPow = 0.15;

    int pitchOff = 0;



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

        private void slideIn() {
            lSlides.setTargetPosition(10);
            rSlides.setTargetPosition(10);

            // Setting power
            lSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
                grab.stop();
                grab.setWristRotation(depoAng);
                armToPos(ready[0], ready[1]);
                return !reached(20);
            }
        }
        public Action readyPos() {
            return new ReadyPos();
        }

        public class ScorePitch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.stop();
                grab.setWristRotation(upAng);
                armToPos(score[0] + pitchOff, score[1] - 2000);
                return !reached(20);
            }
        }
        public Action scorePitch() {
            return new ScorePitch();
        }

        public class ScorePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.stop();
                grab.setWristRotation(upAng);
                armToPos(score[0] + pitchOff, score[1]);
                return !reached(20);
            }
        }
        public Action scorePos() {
            return new ScorePos();
        }

        public class PitchForward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(0.2);
                grab.setWristRotation(depoAng);
                armToPos(score[0], score[1]);
                return !reached(20);
            }
        }
        public Action pitchForward() {
            return new PitchForward();
        }

        public class Grab1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(upAng);
                armToPos(grabArm1[0], grabArm1[1]);
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
                armToPos(grabArm1[0], grabArm1[1] + 900);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action grab1out() {
            return new Grab1Out();
        }

        public class Grab2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(upAng);
                armToPos(grabArm2[0], grabArm2[1]);
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
                armToPos(grabArm2[0], grabArm2[1] + 900);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action grab2out() {
            return new Grab2Out();
        }

        public class Grab3 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(upAng);
                armToPos(grabArm3[0], grabArm3[1]);
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
                armToPos(grabArm3[0], grabArm3[1] + 800);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action grab3out() {
            return new Grab3Out();
        }

        public class Depo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.setWristRotation(depoAng);
                return false;
            }
        }
        public Action depo () {
            return new Depo();
        }

        public class Outtake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.outtake(1);
                return false;
            }
        }
        public Action outtake () {
            return new Outtake();
        }

        public class In implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.slideIn();
                return !reached(20);
            }
        }
        public Action in () {
            return new In();
        }

        public class WristUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.setWristRotation(upAng);
                return false;
            }
        }
        public Action wristUp () {
            return new WristUp();
        }

        public class WristDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.setWristRotation(depoAng);
                return false;
            }
        }
        public Action wristDown () {
            return new WristUp();
        }

    }

    //RR
    Pose2d initialPose;
    PinpointDrive drive;

    AutoArm arm;
    ElapsedTime tm1;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(-62, 35, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, initialPose);

        arm = new AutoArm(hardwareMap);
        tm1 = new ElapsedTime();

        TrajectoryActionBuilder first = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(move1[0], move1[1]));


        telemetry.addData("Program", "States Left");
        telemetry.update();

        //TODO: Change to initialization later
        arm.armToPos(ready[0], ready[1]);
        grab.setWristRotation(0.1);
        grab.stop();
        grab.release();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        first.build(),
                        arm.scorePitch()
                )
        );

        TrajectoryActionBuilder toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoringPos[0] + 2, scoringPos[1] + 7), scoringPos[2] + Math.toRadians(5));

        TrajectoryActionBuilder waiter = drive.actionBuilder(drive.pose)
                .waitSeconds(1);

        Actions.runBlocking(
                new SequentialAction(
                        toScore.build(),
                        arm.scorePos(),
                        arm.depo()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 400) {
            time = tm1.milliseconds();
        }

        Actions.runBlocking(
                new SequentialAction(
                        arm.outtake()
                )
        );


        waiter = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5);

        Actions.runBlocking(
                new SequentialAction(
                        waiter.build()
                )
        );

        TrajectoryActionBuilder toGet1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(pickPos1[0], pickPos1[1]), pickPos1[2]);

        Actions.runBlocking(
                new SequentialAction(
                        arm.scorePitch(),
                        new ParallelAction(
                                arm.grab1(),
                                toGet1.build()
                        ),
                        arm.grab1out()
                )
        );

        toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoringPos[0] - 3, scoringPos[1] + 3), scoringPos[2]);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                arm.scorePitch(),
                                toScore.build()
                        ),
                        arm.scorePos(),
                        arm.depo()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 400) {
            time = tm1.milliseconds();
        }

        Actions.runBlocking(
                new SequentialAction(
                        arm.outtake()
                )
        );

        waiter = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5);

        Actions.runBlocking(
                new SequentialAction(
                        waiter.build()

                )
        );

        TrajectoryActionBuilder toGet2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(pickPos2[0], pickPos2[1]), pickPos2[2]);

        Actions.runBlocking(
                new SequentialAction(
                        arm.scorePitch(),
                        new ParallelAction(
                                arm.grab2(),
                                toGet2.build()
                        ),
                        arm.grab2out()
                )
        );

        toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoringPos[0] - 3, scoringPos[1] + 3), scoringPos[2]);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                arm.scorePitch(),
                                toScore.build()
                        ),
                        arm.scorePos(),
                        arm.depo()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 400) {
            time = tm1.milliseconds();
        }

        Actions.runBlocking(
                new SequentialAction(
                        arm.outtake()
                )
        );

        waiter = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5);

        Actions.runBlocking(
                new SequentialAction(
                        waiter.build()

                )
        );

        TrajectoryActionBuilder toGet3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(pickPos3[0], pickPos3[1]), pickPos3[2]);

        Actions.runBlocking(
                new SequentialAction(
                        arm.scorePitch(),
                        new ParallelAction(
                                arm.grab3(),
                                toGet3.build()
                        ),
                        arm.grab3out()
                )
        );

        toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoringPos[0] - 3, scoringPos[1] + 3), scoringPos[2]);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                arm.scorePitch(),
                                toScore.build()
                        ),
                        arm.scorePos(),
                        arm.depo()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 400) {
            time = tm1.milliseconds();
        }

        Actions.runBlocking(
                new SequentialAction(
                        arm.outtake()
                )
        );

        waiter = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5);

        Actions.runBlocking(
                new SequentialAction(
                        waiter.build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        arm.scorePitch(),
                        arm.in(),
                        arm.readyPos()
                )
        );


        /* Seems tedious to use
        Actions.runBlocking(
                new MecanumDrive.TurnAction(
                        new TimeTurn(drive.pose, Math.toRadians(90), add turn constraint here)
                )
        );
        */
    }
}
