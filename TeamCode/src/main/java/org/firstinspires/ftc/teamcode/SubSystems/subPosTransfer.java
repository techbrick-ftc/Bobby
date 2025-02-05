package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class subPosTransfer extends LinearOpMode {
    static double autoAng;

    public static double getAngle(){
        return autoAng;
    }

    public static void setAngle(double ang){
        autoAng = ang;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
