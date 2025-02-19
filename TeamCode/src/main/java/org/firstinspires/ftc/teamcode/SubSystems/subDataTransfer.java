package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class subDataTransfer extends LinearOpMode {
    static double autoAng;
    static boolean isRedTeam;
    static Pose2D pose;

    public static double getAngle(){
        return autoAng;
    }

    public static boolean getTeam(){
        return isRedTeam;
    }

    public static void setTeam(boolean team){
        isRedTeam = team;
    }

    public static void setAngle(double ang){
        autoAng = ang;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
