package org.firstinspires.ftc.teamcode.SubSystems;
import android.animation.ArgbEvaluator;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class subLED {

    RevBlinkinLedDriver blinkin;
    static int state;

    public subLED(HardwareMap hardwareMap){
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin1");
        setIdle();
    }

    public void setBlue(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        state = 1;
    }

    public void setRed(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        state = 1;
    }

    public void setYellow(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        state = 1;
    }

    public void setAuto(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
    }

    public void setFire(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
        state = 2;
    }

    public void setIdle(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
        state = 0;
    }

    public int getState(){
        return state;
    }
}
