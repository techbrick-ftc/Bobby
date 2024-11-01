package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class subLift {

    public DcMotorEx lift;
    public int liftMin = 0;
    public int liftMax = 7900;

    public subLift(HardwareMap hardwareMap) {

        //TODO: Setup configurations later
        lift = hardwareMap.get(DcMotorEx.class, "LA");

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorEx.Direction.REVERSE);



    }
    public void setLift(int pos, double pow) {
        if (pos >= liftMin && pos <= liftMax) {
            lift.setTargetPosition(pos);
        }
        else if (pos < liftMin){
            lift.setTargetPosition(liftMin);
        }
        else{
            lift.setTargetPosition(liftMin);
        }
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setPower(pow);
    }

    public void runLift(double pow, int dir){
        if (dir == 0 && lift.getCurrentPosition() > liftMin){
            lift.setPower(-pow);
        }
        else if (dir == 1 && lift.getCurrentPosition() < liftMax){
            lift.setPower(pow);
        }
        else{
            lift.setPower(0);
        }

    }


    public boolean reached(DcMotorEx motor, int tol) {
        return Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) < tol;
    }
}
