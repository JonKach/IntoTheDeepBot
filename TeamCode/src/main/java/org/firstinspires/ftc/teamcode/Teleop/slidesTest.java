package org.firstinspires.ftc.teamcode.Teleop;

import android.os.Debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.InfernBot;

@TeleOp
@Config
public class slidesTest extends Teleop {
    DcMotor slides;
    DcMotorEx tilt;
    Servo pivotServo;
    Servo clawServo;
    InfernBot bot;

    Servo lockingServo;

    PIDController controller;

    public static double p = 0.008, i = 0.32, d = 0.001;
    public static double f = -0.2;

    public static double targetPos = 0;

    public static double tiltMotorTicksPerDegree = (-727 - -153) / 90.0;

    public void init() {
        slides = hardwareMap.get(DcMotor.class, "slidesMotor");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tilt = hardwareMap.get(DcMotorEx.class, "tiltMotor");
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt.setDirection(DcMotorSimple.Direction.REVERSE);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setTargetPosition(0);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        lockingServo = hardwareMap.get(Servo.class, "lockingServo");
        bot = new InfernBot(hardwareMap, gamepad1,gamepad2, telemetry);
    }

    @Override
    public void loop() {
//        lockingServo.setPosition(0.25);
        if(gamepad2.a) {
            slides.setPower(1.0);
        } else if (gamepad2.b){
            slides.setPower(-1.0);
        } else {
            slides.setPower(0.0);
        }

        if(gamepad2.left_trigger > 0.3) {
            tilt.setTargetPosition(688);
            tilt.setVelocity(90, AngleUnit.DEGREES);
        } else if(gamepad2.right_trigger > 0.3) {
            tilt.setTargetPosition(0);
            tilt.setVelocity(90, AngleUnit.DEGREES);
        }

        //PIDF
//        telemetry.addData("Angle", tilt.getCurrentPosition()/tiltMotorTicksPerDegree); //add 68.676
//        controller.setPID(p, i, d);
//        int armPos = tilt.getCurrentPosition();
//        double pid = controller.calculate(armPos, targetPos);
//        double ff = Math.cos(Math.toRadians((targetPos/tiltMotorTicksPerDegree) + 68.676)) * f;
//
//        double power = pid + ff;
//        telemetry.addData("ff", ff);
//        telemetry.addData("power", power);
//        tilt.setPower(power);
//
//        telemetry.addData("pos", tilt.getCurrentPosition());
//        telemetry.addData("target", targetPos);
//        telemetry.update();

        if(gamepad2.dpad_up) {
            pivotServo.setPosition(0.1);
        } else if(gamepad2.dpad_down) {
            pivotServo.setPosition(0.5);
        }
        if(gamepad2.left_bumper) {
            clawServo.setPosition(0.35);
        } else if(gamepad2.right_bumper){
            clawServo.setPosition(0);
        }
        if(gamepad2.dpad_left) {
            lockingServo.setPosition(0.1);
        } else if(gamepad2.dpad_right) {
            lockingServo.setPosition(0.25);
        }


        telemetry.addData("Slides", slides.getCurrentPosition());
        telemetry.addData("Tilt", tilt.getCurrentPosition());
        //7500 lift max

        bot.chassis.fieldCentricDrive();
    }
}
