package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.InfernBot;

public class RedLeftAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        InfernBot robot = new InfernBot(hardwareMap, gamepad1, gamepad2, telemetry);
        int turnAngle = 0;
        int driveTicks = 0; //increment driveticks by relative inches * ticks per in whenever you make a change
        int relativeInches = 0;
        int ticksPerIn = 0;
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            robot.chassis.autoTurn(turnAngle);
            robot.chassis.autoDrive(driveTicks + relativeInches * ticksPerIn);
            //NGL THINK ABOUT HIS
        }
    }
}
