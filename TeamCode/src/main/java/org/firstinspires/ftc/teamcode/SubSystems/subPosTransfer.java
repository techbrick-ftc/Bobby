package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@Autonomous
public class subPosTransfer extends LinearOpMode{
    static double ang;


    public double getAngle(){
        return ang;
    }

    public void setAngle(double angle){
        ang = angle;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
