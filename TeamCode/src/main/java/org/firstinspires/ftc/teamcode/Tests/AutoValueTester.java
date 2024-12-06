package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.SubSystems.subGrab;
import org.firstinspires.ftc.teamcode.SubSystems.subLift;
import org.firstinspires.ftc.teamcode.SubSystems.subShoulder;

import java.util.Locale;

@TeleOp(name = "Auto Value Tester")
public class AutoValueTester extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    double oldTime = 0;

    int extendDist = 0;
    int rotateDist = 0;

    double defGrabPow = 0.9;
    double defShPow = 0.5;
    double defLiftPow = 5.0;

    subShoulder should = null;
    subLift lift = null;
    subGrab grab = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        should = new subShoulder(hardwareMap);
        lift = new subLift(hardwareMap);
        grab = new subGrab(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(0, 0); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        //odo.setEncoderResolution(13.26291192);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
            odo.update();

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */

            if (Math.abs(gamepad1.right_stick_y) > .05) {
                should.setSlidesOverride((int)(should.lSlides.getCurrentPosition() + -50 * gamepad1.right_stick_y), defShPow);
            }

            if (Math.abs(gamepad1.left_stick_y) > .05) {
                should.setShldOverride((int)(should.shoulder.getCurrentPosition() + -50 * gamepad1.left_stick_y), defShPow);
            }

            if (gamepad1.dpad_down){
                lift.runLiftOverride(defLiftPow, 0);
            }

            else if (gamepad1.dpad_up){
                lift.runLiftOverride(defLiftPow, 1);
            }
            else{
                lift.runLiftOverride(0, 2);
            }

            if (gamepad1.right_bumper){
                grab.intake(defGrabPow);
                telemetry.addData("Intake", true);
            }
            else if (gamepad1.left_bumper){
                grab.outtake(defGrabPow, defGrabPow);
                telemetry.addData("Outtake", true);
            }
            else{
                grab.stop();
            }

            telemetry.addData("Right Slide Value: ", should.rSlides.getCurrentPosition());
            telemetry.addData("Left Slide Value: ", should.lSlides.getCurrentPosition());
            telemetry.addData("Shoulder Value: ", should.shoulder.getCurrentPosition());
            telemetry.addData("Lift Value: ", lift.lift.getCurrentPosition());

            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();

        }
    }
}
