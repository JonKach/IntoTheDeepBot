package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.InfernBot;

@Config
@TeleOp
public class pidTUNE extends OpMode {
    InfernBot robot;
    public static double angle = 90;
    public static int driveInches = 0;
    public int overallticks = 0;
    public static boolean updateDrive = true;
    public static double ticksPerIn = 40.39;
    @Override
    public void init() {
        robot = new InfernBot(hardwareMap, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void loop() {
        telemetry.addData("targetTicks", overallticks);
        if(updateDrive) {
            overallticks += driveInches * ticksPerIn;
            updateDrive = false;
        } else {
            robot.chassis.autoDrive(overallticks);
        }
        robot.arm.actuateArm();
        robot.chassis.imuTelemetry(telemetry);
    }
}
