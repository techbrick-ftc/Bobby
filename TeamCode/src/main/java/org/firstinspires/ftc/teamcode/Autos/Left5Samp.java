package org.firstinspires.ftc.teamcode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subDataTransfer;

@Config
@Autonomous(name = "5 Samp Auto")
public class Left5Samp extends LinearOpMode {

    //TODO: Shoulder motor initializations
    DcMotorEx shoulder;
    DcMotorEx lSlides;
    DcMotorEx rSlides;

    //TODO: Sensor initializations
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    //TODO: Tune robot positions here
    // x, y, heading
    double[] move1 = {-64, 35, Math.toRadians(90)};
    double[] scoringPos = {-44, 47, Math.toRadians(135)};
    double[] picksX = {-30, -22, -22, -4};
    double[] picksY = {34, 42, 48, 29};
    double[] picksAng = {Math.toRadians(65), Math.toRadians(84), Math.toRadians(85), Math.toRadians(-90)};

    //TODO: Turn arm positions here
    // pitch, slides
    int[] ready = {1290, 10};
    int[] score = {1690, 3100};
    int[] grabSlides = {150, 200, 600, 20};
    int grabPitch = 10;

    //TODO: Grab
    subGrab grab;
    double upAng = 0.56;
    double grabAng = 0.48;
    double backAng = 0.70;
    double depoAng = 0.45; //Used to be 0.26
    double inAng = 0.34;
    double inPow = 0.15;

    int pitchOff = 100;



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

        private void armToPos(int pitch, int slide, double sldPow) {

            // Setting position
            shoulder.setTargetPosition(pitch);
            lSlides.setTargetPosition(slide);
            rSlides.setTargetPosition(slide);

            // Setting power
            shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1.0);
            lSlides.setPower(sldPow);
            rSlides.setPower(sldPow);

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

        private boolean pitchReached(int tol) {
            return Math.abs(shoulder.getCurrentPosition() - shoulder.getTargetPosition()) < tol;
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

        public class FirstScorePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(0.1);
                grab.setWristRotation(upAng);
                armToPos(score[0], score[1]);
                return !reached(20);
            }
        }
        public Action firstScorePos() {
            return new FirstScorePos();
        }

        public class ScorePitch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(0.1);
                grab.setWristRotation(upAng);
                armToPos(score[0] - 1000, 20);
                return !pitchReached(50);
            }
        }
        public Action scorePitch() {
            return new ScorePitch();
        }

        public class OuttakeOnPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.stop();
                grab.setWristRotation(upAng);
                armToPos(score[0], score[1] - 300);
                return !reached(50);
            }
        }
        public Action outtakeOnPos() {
            return new OuttakeOnPos();
        }

        public class ScorePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.outtake(1);
                grab.setWristRotation(upAng);
                armToPos(score[0], score[1]);
                return !reached(20);
            }
        }
        public Action scorePos() {
            return new ScorePos();
        }

        public class SlowScorePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.stop();
                grab.setWristRotation(upAng);
                armToPos(score[0], score[1], 0.4);
                return !reached(20);
            }
        }
        public Action slowScorePos() {
            return new SlowScorePos();
        }

        public class BackReady implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(backAng);
                armToPos(score[0], score[1] - 800);
                return !reached(100);
            }
        }
        public Action backReady() {
            return new BackReady();
        }

        public class Grab1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(grabAng);
                armToPos(-15, grabSlides[0]);
                return !reached(40);
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
                armToPos(-30, grabSlides[0] + 900);
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
                grab.setWristRotation(grabAng);
                armToPos(grabPitch, grabSlides[1]);
                return !reached(50);
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
                armToPos(grabPitch, grabSlides[1] + 600);
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
                grab.setWristRotation(grabAng);
                armToPos(grabPitch, grabSlides[2] + 300);
                return !reached(50);
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
                armToPos(grabPitch, grabSlides[2] + 800);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action grab3out() {
            return new Grab3Out();
        }

        public class Grab4 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(grabAng);
                armToPos(grabPitch, grabSlides[3]);
                return !reached(50);
            }
        }
        public Action grab4() {
            return new Grab4();
        }

        public class Grab4Out implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(inAng);
                armToPos(grabPitch, grabSlides[3] + 1500);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action grab4out() {
            return new Grab4Out();
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
    MinMax linAccel;
    subDataTransfer trans;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(-62, 35, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, initialPose);

        arm = new AutoArm(hardwareMap);
        tm1 = new ElapsedTime();

        trans = new subDataTransfer();

        linAccel = new MinMax(-80, 90);

        Action[] grabList = {arm.grab1(), arm.grab2(), arm.grab3()};
        Action[] outList = {arm.grab1out(), arm.grab2out(), arm.grab3out()};

        TrajectoryActionBuilder toGet;

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
                        first.build()
                )
        );

        TrajectoryActionBuilder toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoringPos[0] + 1, scoringPos[1] + 8), scoringPos[2] + Math.toRadians(5));

        Actions.runBlocking(
                new ParallelAction(
                        toScore.build(),

                        new SequentialAction(
                                arm.outtakeOnPos(),
                                arm.scorePos()
                        )
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        arm.outtake()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 150 && opModeIsActive()) {
            time = tm1.milliseconds();
        }

        TrajectoryActionBuilder toGet1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(picksX[0], picksY[0]), picksAng[0]);

        Actions.runBlocking(

                new ParallelAction(
                        new SequentialAction(
                                arm.backReady(),
                                arm.grab1()
                        ),

                        toGet1.build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        arm.grab1out()
                )
        );

        toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoringPos[0] - 3, scoringPos[1] + 3), scoringPos[2]);

        Actions.runBlocking(

                new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        arm.scorePitch(),
                                        arm.outtakeOnPos(),
                                        arm.scorePos()
                                ),
                                toScore.build()
                        ),

                        arm.outtake()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 100 && opModeIsActive()) {
            time = tm1.milliseconds();
        }

        TrajectoryActionBuilder toGet2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(picksX[1], picksY[1]), picksAng[1]);

        Actions.runBlocking(

                new ParallelAction(
                        new SequentialAction(
                                arm.backReady(),
                                arm.grab2()
                        ),

                        toGet2.build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        arm.grab2out()
                )
        );

        toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoringPos[0] - 3, scoringPos[1] + 3), scoringPos[2]);

        Actions.runBlocking(

                new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        arm.scorePitch(),
                                        arm.outtakeOnPos(),
                                        arm.scorePos()
                                ),
                                toScore.build()
                        ),

                        arm.outtake()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 100 && opModeIsActive()) {
            time = tm1.milliseconds();
        }

        TrajectoryActionBuilder toGet3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(picksX[2], picksY[2]), picksAng[2]);

        Actions.runBlocking(

                new ParallelAction(
                        new SequentialAction(
                                arm.backReady(),
                                arm.grab3()
                        ),

                        toGet3.build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        arm.grab3out()
                )
        );

        toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoringPos[0] - 3, scoringPos[1] + 3), scoringPos[2]);

        Actions.runBlocking(

                new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        arm.scorePitch(),
                                        arm.outtakeOnPos(),
                                        arm.scorePos()
                                ),
                                toScore.build()
                        ),

                        arm.outtake()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 100 && opModeIsActive()) {
            time = tm1.milliseconds();
        }

        TrajectoryActionBuilder toGet4 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(picksX[3], picksY[3] + 24), picksAng[3],
                        new VelConstraint() {
                            @Override
                            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                                return 120;
                            }
                        },

                        new AccelConstraint() {
                            @NonNull
                            @Override
                            public MinMax minMaxProfileAccel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                                return linAccel;
                            }
                        })
                .strafeToConstantHeading(new Vector2d(picksX[3], picksY[3] + 3))
                ;

        Actions.runBlocking(

                new ParallelAction(
                        new SequentialAction(
                                arm.backReady(),
                                arm.grab4()
                        ),

                        toGet4.build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        arm.grab4out()
                )
        );

        toScore = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(picksX[3], picksY[3] + 24), picksAng[3], new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 120;
                        }
                    },

                new AccelConstraint() {
                    @NonNull
                    @Override
                    public MinMax minMaxProfileAccel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return linAccel;
                    }
                })
                .strafeToLinearHeading(new Vector2d(scoringPos[0] - 1, scoringPos[1] + 6), scoringPos[2]);

        Actions.runBlocking(

                new SequentialAction(

                        new ParallelAction(
                                toScore.build(),
                                arm.readyPos()
                        ),
                        arm.outtakeOnPos(),
                        arm.scorePos(),
                        arm.outtake()
                )
        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 100 && opModeIsActive()) {
            time = tm1.milliseconds();
        }

        trans.setAngle(drive.pose.heading.toDouble());
    }
}
