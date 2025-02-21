package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Chassis;
import org.firstinspires.ftc.teamcode.Hardware.InfernBot;

@Autonomous
@Config
public class RedLeftAuton extends LinearOpMode {

    public static int first_forward = -25;
    public static int second_move = -20;

    @Override
    public void runOpMode() throws InterruptedException {
        InfernBot.ranAuto = false;
        InfernBot robot = new InfernBot(hardwareMap, gamepad1, gamepad2, telemetry);
        int turnAngle = 180; //starting angle
        int overallDriveTicks = 0; //starting drive ticks;
        int driveTicksTolerance = 120;
        int angleDegreesTolerance = 5;
        int currState = 0;
        boolean updateDriveTicks = true;
        boolean driving = false;
        boolean turning = false;
        boolean timerHasBeenReset = false;
        boolean turnHasStarted = false;

        ElapsedTime inToleranceTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean inTolerance = false;

        waitForStart();
        InfernBot.ranAuto = true;
        while(opModeIsActive() && !isStopRequested()) {
            robot.arm.actuateArm(true);
            robot.chassis.imuTelemetry(telemetry);
            telemetry.addData("turnAngle", turnAngle);
            telemetry.addData("overallTicks", overallDriveTicks);
            telemetry.update();
            if(turning) robot.chassis.autoTurn(turnAngle);
            if(driving) robot.chassis.autoDrive(overallDriveTicks);

            switch(currState) {
                case 0:
                    if(robot.arm.currTiltState == Arm.TiltState.OUTTAKE) currState++; //waits until arm init positions are done
                    break;
                case 1: //DRIVE TOWARD THE NET ZONE
                    if(!timerHasBeenReset) {
                        robot.chassis.resetDriveTimer(); //may have to do before every pid on6
                        timerHasBeenReset = true;
                    }
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
                        } else if(inToleranceTimer.time() > 800) {
                            currState++;
                            inTolerance = false; //resets for next case
                            updateDriveTicks = true; //resets for next case
                            timerHasBeenReset = false;
                            turnHasStarted = false;
                        }
                    } else {
                        inTolerance = false;
                    }
                    break;
                case 2:
                    driving = false;
                    if(!timerHasBeenReset) {
                        robot.chassis.stopDrive();
                        robot.chassis.resetTurnTimer();
                        timerHasBeenReset = true;
                    }
                    turning = true;
                    turnAngle = 45;
                    if(!turnHasStarted) {
                        inToleranceTimer.reset();
                        turnHasStarted = true;
                    }
                    if(inToleranceTimer.time() > 1000) {
                        currState++;
                        inTolerance = false; //resets for next case
                        updateDriveTicks = true; //resets for next case
                        timerHasBeenReset = false;
                        turnHasStarted = false;
                    }
                    break;
                    //may want to do a turning state to make sure its aligned still, then lift next state
                    //now just do EXACTLY what the buttons do in their own functions, so the states will work
                    //ALWAYS reset necesssary booleans when ending a case
                case 3:
                    if(!timerHasBeenReset) {
                        robot.chassis.stopDrive();
                        robot.chassis.resetDriveTimer(); //may have to do before every pid on6
                        timerHasBeenReset = true;
                    }
                    driving = true; //turn on drivePID
                    turning = false;
                    if(updateDriveTicks) {
                        overallDriveTicks += second_move * robot.chassis.driveTicksPerIn;
                        updateDriveTicks = false;
                    }
                    if(Math.abs(robot.chassis.avgCurrDriveTicks - overallDriveTicks) < driveTicksTolerance) {
                        if(!inTolerance) {
                            inTolerance = true;
                            inToleranceTimer.reset();
                        } else if(inToleranceTimer.time() > 800) {
                            currState++;
                            inTolerance = false; //resets for next case
                            updateDriveTicks = true; //resets for next case
                            timerHasBeenReset = false;
                            turnHasStarted = false;
                        }
                    } else {
                        inTolerance = false;
                    }
                    break;
                case 4:
                    driving = false;
                    turning = false;
                    if(!timerHasBeenReset) {
                        robot.chassis.stopDrive();
                        robot.chassis.resetTurnTimer();
                        robot.chassis.resetDriveTimer();
                        timerHasBeenReset = true;
                    }
                    robot.arm.pressY();
                    if(robot.arm.slidesMotor.getCurrentPosition() > robot.arm.extendPos - 50) {
                        robot.arm.openClaw();
                        if(!inTolerance) {
                            inTolerance = true;
                            inToleranceTimer.reset();
                        } else if(inToleranceTimer.time() > 300) {
                            currState++;
                            inTolerance = false; //resets for next case
                            updateDriveTicks = true; //resets for next case
                            timerHasBeenReset = false;
                            turnHasStarted = false;
                        }
                    }
                    break;
                case 5:
                    driving = false;
                    turning = false;
                    if(!timerHasBeenReset) {
                        robot.chassis.stopDrive();
                        robot.chassis.resetTurnTimer();
                        robot.chassis.resetDriveTimer();
                        timerHasBeenReset = true;
                    }
                    robot.arm.pressX();
                    if(robot.arm.currTiltState == Arm.TiltState.INTAKE) {
                        currState++;
                        inTolerance = false; //resets for next case
                        updateDriveTicks = true; //resets for next case
                        timerHasBeenReset = false;
                        turnHasStarted = false;
                    }
                    break;
                case 6:
                    driving = false;
                    if(!timerHasBeenReset) {
                        robot.chassis.stopDrive();
                        robot.chassis.resetTurnTimer();
                        robot.chassis.resetDriveTimer();
                        timerHasBeenReset = true;
                    }
                    turning = true;
                    turnAngle = -15;
                    if(!turnHasStarted) {
                        inToleranceTimer.reset();
                        turnHasStarted = true;
                    }
                    if(inToleranceTimer.time() > 1000) {
                        currState++;
                        inTolerance = false; //resets for next case
                        updateDriveTicks = true; //resets for next case
                        timerHasBeenReset = false;
                        turnHasStarted = false;
                    }
                    break;
                case 7:
                    driving = false;
                    turning = false;
                    if(!timerHasBeenReset) {
                        robot.chassis.stopDrive();
                        robot.chassis.resetTurnTimer();
                        robot.chassis.resetDriveTimer();
                        timerHasBeenReset = true;
                    }
                    robot.arm.pressY();
                    if(robot.arm.slidesMotor.getCurrentPosition() > robot.arm.intakeExtendPos - 50) {
                        robot.arm.rotateClawDown();
                        if(!inTolerance) {
                            inTolerance = true;
                            inToleranceTimer.reset();
                        } else if(inToleranceTimer.time() > 500) {
                            currState++;
                            inTolerance = false; //resets for next case
                            updateDriveTicks = true; //resets for next case
                            timerHasBeenReset = false;
                            turnHasStarted = false;
                        }
                    }
                    break;
                case 8:
                    robot.arm.closeClaw();
                    if(!turnHasStarted) {
                        inToleranceTimer.reset();
                        turnHasStarted = true;
                    }
                    if(inToleranceTimer.time() > 300) {
                        currState++;
                        inTolerance = false; //resets for next case
                        updateDriveTicks = true; //resets for next case
                        timerHasBeenReset = false;
                        turnHasStarted = false;
                    }
                    break;
                case 9:
                    robot.arm.rotateClawUp();
                    if(!turnHasStarted) {
                        inToleranceTimer.reset();
                        turnHasStarted = true;
                    }
                    if(inToleranceTimer.time() > 300) {
                        currState++;
                        inTolerance = false; //resets for next case
                        updateDriveTicks = true; //resets for next case
                        timerHasBeenReset = false;
                        turnHasStarted = false;
                    }
                    break;
                case 10:
                    driving = false;
                    turning = false;
                    if(!timerHasBeenReset) {
                        robot.chassis.stopDrive();
                        robot.chassis.resetTurnTimer();
                        robot.chassis.resetDriveTimer();
                        timerHasBeenReset = true;
                    }
                    robot.arm.pressB();
                    if(robot.arm.currTiltState == Arm.TiltState.OUTTAKE) {
                        currState++;
                        inTolerance = false; //resets for next case
                        updateDriveTicks = true; //resets for next case
                        timerHasBeenReset = false;
                        turnHasStarted = false;
                    }
                    break;
                case 11:
                    driving = false;
                    if(!timerHasBeenReset) {
                        robot.chassis.stopDrive();
                        robot.chassis.resetTurnTimer();
                        timerHasBeenReset = true;
                    }
                    turning = true;
                    turnAngle = 45;
                    if(!turnHasStarted) {
                        inToleranceTimer.reset();
                        turnHasStarted = true;
                    }
                    if(inToleranceTimer.time() > 1000) {
                        currState++;
                        inTolerance = false; //resets for next case
                        updateDriveTicks = true; //resets for next case
                        timerHasBeenReset = false;
                        turnHasStarted = false;
                    }
                    break;
                case 12:
                    driving = false;
                    turning = false;
                    if(!timerHasBeenReset) {
                        robot.chassis.stopDrive();
                        robot.chassis.resetTurnTimer();
                        robot.chassis.resetDriveTimer();
                        timerHasBeenReset = true;
                    }
                    robot.arm.pressY();
                    if(robot.arm.slidesMotor.getCurrentPosition() > robot.arm.extendPos - 50) {
                        robot.arm.openClaw();
                        if(!inTolerance) {
                            inTolerance = true;
                            inToleranceTimer.reset();
                        } else if(inToleranceTimer.time() > 300) {
                            currState++;
                            inTolerance = false; //resets for next case
                            updateDriveTicks = true; //resets for next case
                            timerHasBeenReset = false;
                            turnHasStarted = false;
                        }
                    }
                    break;
                case 13:
                    driving = false;
                    turning = false;
                    if(!timerHasBeenReset) {
                        robot.chassis.stopDrive();
                        robot.chassis.resetTurnTimer();
                        robot.chassis.resetDriveTimer();
                        timerHasBeenReset = true;
                    }
                    robot.arm.pressB();
                    if(robot.arm.currTiltState == Arm.TiltState.INTAKE) {
                        currState++;
                        inTolerance = false; //resets for next case
                        updateDriveTicks = true; //resets for next case
                        timerHasBeenReset = false;
                        turnHasStarted = false;
                    }
                    break;
            }
        }
    }
}
