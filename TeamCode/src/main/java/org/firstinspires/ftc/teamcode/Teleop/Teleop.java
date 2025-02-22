package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.InfernBot;
@TeleOp
public class Teleop extends OpMode {
    InfernBot robot;
    @Override
    public void init() {
        robot = new InfernBot(hardwareMap, gamepad1, gamepad2, telemetry);
    }

    public void init_loop() {
        telemetry.addData("ranAuto", InfernBot.ranAuto);
        telemetry.update();
        if(gamepad2.a) {
            InfernBot.ranAuto = false;
            robot = new InfernBot(hardwareMap, gamepad1, gamepad2, telemetry);
        }
    }
    @Override
    public void loop() {
        robot.chassis.fieldCentricDrive();
        robot.arm.actuateArm(false);
        robot.chassis.imuTelemetry(telemetry);
    }
}
