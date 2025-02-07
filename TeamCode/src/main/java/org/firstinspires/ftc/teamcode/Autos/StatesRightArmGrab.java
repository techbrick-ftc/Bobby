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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subDataTransfer;

@Config
@Autonomous(name = "Right push arm test")
public class StatesRightArmGrab extends LinearOpMode {

    //TODO: Shoulder motor initializations
    DcMotorEx shoulder;
    DcMotorEx lSlides;
    DcMotorEx rSlides;

    double scoreX = -28;
    double[] scoreY = {-6, -3, 0, 3, 6};
    double[] pushPos = {-54, -62, -72};

    //TODO: Tune robot positions here
    // x, y, heading
    double[] initPos = {-62, -22, Math.toRadians(90)};
    double[] clearPos = {-32, -42, Math.toRadians(-72)};
    double[] toGrab = {-58, -44, Math.toRadians(180)};
    double[] firstSplineTo = {-30, -42, Math.toRadians(-90)};
    double[] depoPos = {-45, -42, Math.toRadians(-135)};

    //TODO: Tune arm positions here
    // pitch, slides
    int[] ready = {1290, 10};
    int[] barInit = {2270, 160};
    int[] barRaise = {2270, 770};
    int[] wall = {720, 10};
    int groundPitch = 5;
    int[] groundSlides = {1100, 20, 500};

    //TODO: Intake angles
    double depoAng = 0.44;
    double wallInAng = 0.09;

    subGrab grab;

    subDataTransfer trans;

    double grabAng = 0.48;
    double inAng = 0.34;
    double upAng = 0.55;

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

        //Drive arm to position
        private void armToPos(int pitch, int slide, double pit, double sld) {

            // Setting position
            shoulder.setTargetPosition(pitch);
            lSlides.setTargetPosition(slide);
            rSlides.setTargetPosition(slide);

            // Setting power
            shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            shoulder.setPower(pit);
            lSlides.setPower(sld);
            rSlides.setPower(sld);

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

        public class PrepWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.setWristRotation(wallInAng);
                grab.release();
                armToPos(wall[0], wall[1]);
                return !reached(20);
            }
        }
        public Action prepWall() {
            return new PrepWall();
        }

        public class MovePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armToPos(wall[0] + 500, wall[1]);
                return !reached(20);
            }
        }
        public Action movePos() {
            return new MovePos();
        }

        public class Release implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.release();
                return false;
            }
        }
        public Action release() {
            return new Release();
        }

        public class Outtake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.setWristRotation(upAng);
                grab.outtake(1);
                return false;
            }
        }
        public Action outtake() {
            return new Outtake();
        }

        public class Grab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.grab();
                return false;
            }
        }
        public Action grab() {
            return new Grab();
        }

        public class Get1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(grabAng);
                armToPos(groundPitch, groundSlides[0]);
                return !reached(10);
            }
        }
        public Action get1() {
            return new Get1();
        }

        public class Get1Out implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(inAng);
                armToPos(groundPitch, groundSlides[0] + 900, 1.0, 0.8);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action get1Out() {
            return new Get1Out();
        }

        public class Get2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(grabAng);
                armToPos(groundPitch, groundSlides[1]);
                return !reached(10);
            }
        }
        public Action get2() {
            return new Get2();
        }

        public class Get2Out implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grab.intake(1);
                grab.setWristRotation(inAng);
                armToPos(groundPitch, groundSlides[1] + 200, 1.0, 0.2);
                return !(grab.checkObjectIn() || reached(20));
            }
        }
        public Action get2Out() {
            return new Get2Out();
        }

    }

    //RR
    Pose2d initialPose;
    PinpointDrive drive;
    ElapsedTime tm1;

    MinMax linAccel;

    AutoArm arm;

    @Override
    public void runOpMode() {

        initialPose = new Pose2d(initPos[0], initPos[1], initPos[2]);
        drive = new PinpointDrive(hardwareMap, initialPose);

        arm = new AutoArm(hardwareMap);
        tm1 = new ElapsedTime();

        trans = new subDataTransfer();

        linAccel = new MinMax(-80, 90);

        arm.armToPos(ready[0], ready[1]);
        grab.setWristRotation(0.1);
        grab.lightGrab();

        TrajectoryActionBuilder score1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(scoreX, scoreY[0]), Math.toRadians(180),
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
                        });

        telemetry.addData("Program", "States Right");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                score1.build(),
                                arm.barInit()
                        ),
                        arm.barScore(),
                        arm.release()
                )
        );

        TrajectoryActionBuilder initials = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(scoreX - 8, scoreY[0]), Math.toRadians(180),
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
                .strafeToLinearHeading(new Vector2d(clearPos[0], clearPos[1]), clearPos[2], new VelConstraint() {
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
                .strafeToLinearHeading(new Vector2d(clearPos[0]+1, clearPos[1]), clearPos[2]-2)
                ;



        Actions.runBlocking(
                new SequentialAction (
                        new ParallelAction(
                                arm.get1(),
                                initials.build()
                        ),
                        arm.get1Out()
                )
        );

        TrajectoryActionBuilder depo = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(depoPos[0], depoPos[1]), depoPos[2],
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
                ;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                arm.get1(),
                                depo.build()
                        ),
                        arm.outtake()
                )

        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 400 && opModeIsActive()) {
            time = tm1.milliseconds();
        }

        TrajectoryActionBuilder toGet2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(clearPos[0] + 6, clearPos[1] + 3), Math.toRadians(-90),
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
                .strafeToLinearHeading(new Vector2d(clearPos[0] + 9, clearPos[1] + 4), Math.toRadians(-90))
                ;

        Actions.runBlocking(
                new SequentialAction (
                        new ParallelAction(
                                arm.get2(),
                                toGet2.build()
                        ),
                        arm.get2Out()
                )
        );

        depo = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(depoPos[0], depoPos[1]), depoPos[2],
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
                ;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                depo.build()
                        ),
                        arm.outtake()
                )

        );

        tm1.reset();
        time = tm1.milliseconds();
        while (time < 400 && opModeIsActive()) {
            time = tm1.milliseconds();
        }


        trans.setAngle(drive.pose.heading.toDouble());
    }
}
