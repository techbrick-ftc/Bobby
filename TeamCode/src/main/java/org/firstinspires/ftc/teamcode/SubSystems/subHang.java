package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Teleops.Main;


public class subHang {

    subShoulder should;
    subLift lift;

    int state = 0;

    double defShPow = 1;
    double defSlPow = 1;
    double defLiftPow = 1;

    int defTol = 20;
    int rotateTol = 50;

    // Shoulder, slides, lift
    int[] init = {4100, 1850, 7950};
    int[] lowHang1 = {4400, 2700, 3900};
    int[] lowHang2 = {4850, 2700, 3900};
    int[] hookSlides = {4850, 2400, 3900};
    int[] pullUp = {4250, 1100, 1500};
    int[] rotate = {2700, 1100, 0};
    int[] pullHigher = {2700, 290, 0};
    int[] finalize = {1800, 20, 0};


    public subHang(HardwareMap hardwareMap){
        should = new subShoulder(hardwareMap);
        lift = new subLift(hardwareMap);
    }

    public void hang(boolean x, boolean b){
        if (state == 0){
            should.setShld(init[0], defShPow);
            should.setSlides(init[1], defSlPow);
            lift.setLift(init[2], defLiftPow);
            state++;
        }
        else if (state == 1 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(lowHang1[0], defShPow);
            should.setSlides(lowHang1[1], defSlPow);
            lift.setLift(lowHang1[2], defLiftPow);
            state++;
        }
        else if (state == 2 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(lowHang2[0], defShPow);
            should.setSlides(lowHang2[1], defSlPow);
            lift.setLift(lowHang2[2], defLiftPow);
            state++;
        }
        else if (state == 3 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(hookSlides[0], defShPow);
            should.setSlides(hookSlides[1], defSlPow);
            lift.setLift(hookSlides[2], defLiftPow);
            state++;
        }
        else if (state == 4 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(pullUp[0], defShPow);
            should.setSlides(pullUp[1], defSlPow);
            lift.setLift(pullUp[2], defLiftPow);
            state++;
        }
        else if (state == 5 && should.reached(should.lSlides, rotateTol) && should.reached(should.shoulder, rotateTol) && lift.reached(lift.lift, rotateTol) && x) {
            should.setShld(rotate[0], defShPow);
            should.setSlides(rotate[1], defSlPow);
            lift.setLift(rotate[2], defLiftPow);
            state++;
        }
        else if (state == 6 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(pullHigher[0], defShPow);
            should.setSlides(pullHigher[1], defSlPow);
            lift.setLift(pullHigher[2], defLiftPow);
            state++;
        }
        else if (state == 7 && should.reached(should.lSlides, defTol) && should.reached(should.shoulder, defTol) && lift.reached(lift.lift, defTol) && x) {
            should.setShld(finalize[0], defShPow);
            should.setSlides(finalize[1], defSlPow);
            lift.setLift(finalize[2], defLiftPow);
            state = 0;
            Main.routine = 0;
        }
        else if (b){
            state = 0;
            Main.routine = 0;
        }
    }



}
