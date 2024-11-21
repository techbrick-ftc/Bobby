package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.subShoulder;

@Autonomous
public class RunningArmInAuto extends LinearOpMode {

    subShoulder should;

    public void runOpMode() {

        should = new subShoulder(hardwareMap);

        waitForStart();

        while(time < 30) {
            should.setShld(3000, 1);
        }
    }
}
