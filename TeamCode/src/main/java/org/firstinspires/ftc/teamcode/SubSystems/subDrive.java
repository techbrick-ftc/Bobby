package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class subDrive {

    // Timer
    ElapsedTime tm1 = new ElapsedTime();

    // IMU
    BNO055IMU imu;
    Orientation orientation;

    // Motor Declarations

    // Drive Motors
    DcMotorEx br;
    DcMotorEx bl;
    DcMotorEx fr;
    DcMotorEx fl;

    boolean driveAllowed = true;

    public double offset = Math.PI;

    // Getting Hardware map from main file and setting up
    public subDrive(HardwareMap hardwareMap) {

        // Motor definitions
        br = hardwareMap.get(DcMotorEx.class, "BRM");
        bl = hardwareMap.get(DcMotorEx.class, "BLM");
        fr = hardwareMap.get(DcMotorEx.class, "FRM");
        fl = hardwareMap.get(DcMotorEx.class, "FLM");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void run(double x_move, double y_move, double rotation_x, double pow) {

        if (Math.abs(x_move) > 0.05 || Math.abs(y_move) > 0.05 || Math.abs(rotation_x) > 0.05) {
            // Orientation
            orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double angle = orientation.secondAngle - offset;

            // Calculating power variables
            double x = x_move * Math.cos(-(angle)) - y_move * Math.sin(-(angle));
            double y = y_move * Math.cos(-(angle)) + x_move * Math.sin(-(angle));

            // Dividing power by a denominator to set maximum output to 1
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation_x), 1);

            double frontLeftPower = (y + x + rotation_x) / denominator;
            double backLeftPower = (y - x + rotation_x) / denominator;
            double frontRightPower = (y - x - rotation_x) / denominator;
            double backRightPower = (y + x - rotation_x) / denominator;

            // Writing the power to the motors
            fl.setPower(frontLeftPower * pow);
            bl.setPower(backLeftPower * pow);
            fr.setPower(frontRightPower * pow);
            br.setPower(backRightPower * pow);
        }

        else {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }
    }

    public double getImu() {
        //TODO: Change secondAngle if necessary. Depends on IMU orientation
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).secondAngle;
    }
}