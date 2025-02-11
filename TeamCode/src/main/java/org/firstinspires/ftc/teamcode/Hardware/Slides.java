package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {

    public static DcMotor slidesMotor;
    public static double slidePower = 1.0;

    public static int RETRACT_POS, EXTEND_POS, SLIGHTLY_EXTEND_POS;
    public int step;
    Gamepad gamepad2;

    public Slides(HardwareMap hardwareMap, Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        //might need an "if isAuton" then reset
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void actuateSlides() {
        //MANUAL
        if(gamepad2.left_trigger > 0.3) {
            slidesMotor.setTargetPosition(slidesMotor.getCurrentPosition() - step);
            slidesMotor.setPower(slidePower);
        } else if(gamepad2.right_trigger > 0.3) {
            slidesMotor.setTargetPosition(slidesMotor.getCurrentPosition() + step);
            slidesMotor.setPower(slidePower);
        }
        //PRESETS
        if(gamepad2.y) {
            slidesMotor.setTargetPosition(EXTEND_POS);
            slidesMotor.setPower(slidePower);
        } else if(gamepad2.a) {
            slidesMotor.setTargetPosition(RETRACT_POS);
            slidesMotor.setPower(slidePower);
        }
    }

    //AUTO and from other classes
    public static void actuateSlidesToPos(int pos) {
        slidesMotor.setTargetPosition(pos);
        slidesMotor.setPower(slidePower);
    }
}
