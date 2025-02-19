package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.InfernBot;

@Autonomous
public class RedLeftAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        InfernBot robot = new InfernBot(hardwareMap, gamepad1, gamepad2, telemetry);
        int turnAngle = -90; //starting angle
        int overallDriveTicks = 0; //starting drive ticks;
        int driveTicksTolerance = 120;
        int currState = 0;
        boolean updateDriveTicks = true;
        boolean driving = false;
        boolean turning = false;

        ElapsedTime inToleranceTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean inTolerance = false;

        int first_forward = 10;

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            robot.arm.actuateArm(true);
            robot.chassis.imuTelemetry(telemetry);
            telemetry.addData("turnAngle", turnAngle);
            telemetry.addData("overallTicks", overallDriveTicks);
            if(turning) robot.chassis.autoTurn(turnAngle);
            if(driving) robot.chassis.autoDrive(overallDriveTicks);

            switch(currState) {
                case 0:
                    if(robot.arm.currTiltState == Arm.TiltState.OUTTAKE) currState++; //waits until arm init positions are done
                    break;
                case 1: //DRIVE TOWARD THE NET ZONE
                    driving = true; //turn on drivePID
                    turning = false;
                    if(updateDriveTicks) {
                        overallDriveTicks += first_forward * robot.chassis.driveTicksPerIn;
                        updateDriveTicks = false;
                    }
                    if(Math.abs(robot.chassis.avgCurrDriveTicks - overallDriveTicks) < driveTicksTolerance) {
                        if(!inTolerance) {
                            inTolerance = true;
                            inToleranceTimer.reset();
                        } else if(inToleranceTimer.time() > 2500) {
                            currState++;
                            inTolerance = false; //resets for next case
                            updateDriveTicks = true; //resets for next case
                        }
                    } else {
                        inTolerance = false;
                    }
                    break;
                case 2:
                    driving = false;
                    turning = false;
                    //may want to do a turning state to make sure its aligned still, then lift next state
                    telemetry.addLine("would have lifted");
                    telemetry.update();
                    //now just do EXACTLY what the buttons do in their own functions, so the states will work
                    //ALWAYS reset necesssary booleans when ending a case
            }
        }
    }
}
