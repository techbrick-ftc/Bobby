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
    double[] move1 = {-62, 36, Math.toRadians(90)};
    double[] scoringPos = {-49, 54, Math.toRadians(135)};
    double[] pickPos1 = {-36, 34, Math.toRadians(60)};
    double[] pickPos2 = {-25, 36, Math.toRadians(90)};
    double[] pickPos3 = {-25, 48, Math.toRadians(90)};

    //TODO: Turn arm positions here
    // pitch, slides
    int[] ready = {1400, 50};
    int[] score = {2600, 3100};
    int[] grabArm1 = {20, 1100};
    int[] grabArm2 = {20, 1700};
    int[] grabArm3 = {20, 2400};

    //TODO: Grab
    subGrab grab;
    double depoAng = 0.3;
    double inAng = 0.34;

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

            depoAng = 0.3;
            inAng = 0.33;
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
                grab.stop();
                grab.setWristRotation(depoAng);
                armToPos(ready[0], ready[1]);
                return !reached(20);
            }
        }
        public Action readyPos() {
            return new ReadyPos();
        }

        public class ScorePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.stop();
                grab.setWristRotation(depoAng);
                armToPos(score[0], score[1]);
                return !reached(20);
            }
        }
        public Action scorePos() {
            return new ScorePos();
        }

        public class Grab1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(inAng);
                armToPos(grabArm1[0], grabArm1[1]);
                return !grab.checkObjectIn();
            }
        }
        public Action grab1() {
            return new Grab1();
        }

        public class Grab2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(inAng);
                armToPos(grabArm2[0], grabArm2[1]);
                return !reached(20);
            }
        }
        public Action grab2() {
            return new Grab2();
        }

        public class Grab3 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(inAng);
                armToPos(grabArm3[0], grabArm3[1]);
                return !reached(20);
            }
        }
        public Action grab3() {
            return new Grab3();
        }

        public class Depo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.outtake(1);
                return grab.checkObjectIn();
            }
        }
        public Action depo () {
            return new Depo();
        }

    }

    //RR
    Pose2d initialPose;
    PinpointDrive drive;

    AutoArm arm;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(-62, 35, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, initialPose);

        arm = new AutoArm(hardwareMap);

        TrajectoryActionBuilder first = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(move1[0], move1[1]));


        telemetry.addData("Program", "States Left");
        telemetry.update();

        //TODO: Change to initialization later
        arm.armToPos(ready[0], ready[1]);
        grab.setWristRotation(depoAng);
        grab.stop();
        grab.release();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        first.build(),
                        arm.scorePos()
                )
        );

        TrajectoryActionBuilder toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoringPos[0] + 2, scoringPos[1] + 2), scoringPos[2]);


        Actions.runBlocking(
                new SequentialAction(
                        toScore.build(),
                        arm.depo()
                )
        );

        TrajectoryActionBuilder toGet1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(pickPos1[0], pickPos1[1]), pickPos1[2]);

        Actions.runBlocking(
                new ParallelAction(
                        toGet1.build(),
                        arm.grab1()
                )
        );

        toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoringPos[0], scoringPos[1]), scoringPos[2]);

        Actions.runBlocking(
                new SequentialAction(
                        arm.scorePos(),
                        toScore.build(),
                        arm.depo()
                )
        );

        TrajectoryActionBuilder waiter = drive.actionBuilder(drive.pose)
                .waitSeconds(2.1);

        Actions.runBlocking(
                new SequentialAction(
                        waiter.build()
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
