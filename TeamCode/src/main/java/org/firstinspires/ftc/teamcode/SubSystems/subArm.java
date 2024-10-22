package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class subArm {

    subShoulder should;
    subGrab grab;

    public int state = 0;
    public int routine = 0;

    double defShPow = 0.5;
    double defSlPow = 0.5;
    int defTol = 20;

    // Positional vars
    //TODO: Setup these values
    int[] ground = {0, 0};
    int[] intakeLow = {0, 0};
    int[] intakeHigh = {200, 0};
    int[] wallHigh = {0, 0};
    int[] wallLow = {0, 0};
    int[] railHigh = {0, 0};
    int[] railLow = {350, 0};
    int[] lowBin = {350, 0};
    int[] highBin = {450, 0};

    public subArm(HardwareMap hardwareMap) {
        should = new subShoulder(hardwareMap);
        grab = new subGrab(hardwareMap);
    }

    public void home(){

    }

    public void bin(boolean lBump, boolean high) {

        if (state == 0) {
            if (high) {
                should.setShld(highBin[0], defShPow);
                should.setSlides(highBin[1], defSlPow);
            }

            else {
                should.setShld(lowBin[0], defSlPow);
                should.setSlides(lowBin[1], defSlPow);
            }
            state++;
        }

        if (state == 1) {
            if (should.reached(should.slides, defTol) && should.reached(should.shoulder, defTol)) {
                state = 0;
                routine = 0;
            }
        }
    }
}
