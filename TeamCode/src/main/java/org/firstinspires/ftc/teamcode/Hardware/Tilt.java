package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;
import java.util.TimerTask;

public class Tilt {
    public DcMotor tiltMotor;
    public double tiltPower = 0.5;

    public int INTAKE, OUTTAKE;
    public int step;
    Gamepad gamepad2;

    Timer timer;
    public Tilt(HardwareMap hardwareMap, Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
        tiltMotor = hardwareMap.get(DcMotor.class, "tiltMotor");
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setTargetPosition(0);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer = new Timer("timer");
    }

    public void actuateTilt() {
        //MANUAL
        if(gamepad2.left_stick_x > -0.2) {
            tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition() - step);
            tiltMotor.setPower(tiltPower);
        } else if(gamepad2.left_stick_x > 0.2){
            tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition() + step);
            tiltMotor.setPower(tiltPower);
        }
        //PRESETS
        if(gamepad2.x) {
            Slides.actuateSlidesToPos(Slides.SLIGHTLY_EXTEND_POS);
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    tiltMotor.setTargetPosition(INTAKE);
                    tiltMotor.setPower(tiltPower);
                }
            }, 400);
        } else if(gamepad2.b) {
            Slides.actuateSlidesToPos(Slides.SLIGHTLY_EXTEND_POS);
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    tiltMotor.setTargetPosition(INTAKE);
                    tiltMotor.setPower(tiltPower);
                }
            }, 400);
        }
    }


}
