package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Arm {

    //MOTORS
    public DcMotorEx tiltMotor, slidesMotor;
    public double slidesPower = 1.0, tiltPower = 1.0;
    public int slideStep = 300, tiltStep;
    public int extendPos = 7450, retractPos = 0, slightlyExtendedPos = 346, intakeExtendPos = 2200, horizontalExtensionLimit = 3450;
    public int lockedTiltPos = 0, outtakeTiltPos = -270, intakeTiltPos = 407;

    //SERVOS
    public Servo lockingServo, pivotServo, clawServo;
    public double pivotInit = 0.15, pivotIntake = 0.5, pivotOuttake = 0.85, clawIntake = 0.4, clawOuttake = 0.0;
    public double open = 0.1, hold = 0.25;

    //STATES
    public ExtendState currExtendState = ExtendState.RETRACTED;
    public TiltState currTiltState = TiltState.IN_TRANSIT;

    public ExtendState reqExtendState = ExtendState.SLIGHTLY_EXTENDED;
    public TiltState reqTiltState = TiltState.OUTTAKE;

    public boolean isManual = false;

    Gamepad gamepad2;
    Telemetry telemetry;
    public Arm(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry) {
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        tiltMotor = hardwareMap.get(DcMotorEx.class, "tiltMotor");
        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        lockingServo = hardwareMap.get(Servo.class, "lockingServo");

        tiltMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setTargetPosition(0);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivotServo.setPosition(pivotInit);
        clawServo.setPosition(clawIntake);
        lockingServo.setPosition(hold);

        telemetry.addData("tiltPos", tiltMotor.getCurrentPosition());
        telemetry.update();
    }

    public void actuateArm() {

        //MUST LET OUT SOME STRING AFTER STARTING SINCE THE HOLD POSITION WILL BREAK STRING
        //IF IT TRIES TO HOLD THE FULL RETRACTED UPRIGHT POSITION WHEN GOING DOWN (SINCE SOME STRING
        //NEEDS TO SPOOL OUT WHEN GOING DOWN
        telemetry.addData("tiltpos", tiltMotor.getCurrentPosition());
        telemetry.addData("REQ_EXTEND", reqExtendState);
        telemetry.addData("REQ_TILT", reqTiltState);
        telemetry.addData("curr_extend", currExtendState);
        telemetry.addData("curr_tilt", currTiltState);

        //MANUAL
        if(gamepad2.left_trigger > 0.3 && slidesMotor.getCurrentPosition() - slideStep >= slightlyExtendedPos) {
            isManual = true;
            slidesMotor.setTargetPosition(slidesMotor.getCurrentPosition() - slideStep);
            slidesMotor.setPower(slidesPower);
        } else if(gamepad2.right_trigger > 0.3) {
            if(!(currTiltState == TiltState.INTAKE && slidesMotor.getCurrentPosition() + slideStep > horizontalExtensionLimit)) {
                isManual = true;
                slidesMotor.setTargetPosition(slidesMotor.getCurrentPosition() + slideStep);
                slidesMotor.setPower(slidesPower);
            }
        }

        if(gamepad2.dpad_down) {
            pivotServo.setPosition(pivotIntake);
        } else if(gamepad2.dpad_up) {
            pivotServo.setPosition(pivotOuttake);
        }
        if(gamepad2.left_bumper) {
            clawServo.setPosition(clawOuttake);
        } else if(gamepad2.right_bumper){
            clawServo.setPosition(clawIntake);
        }

//        if(gamepad2.left_stick_x > -0.2) {
//            tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition() - tiltStep);
//            tiltMotor.setPower(tiltPower);
//        } else if(gamepad2.left_stick_x > 0.2){
//            tiltMotor.setTargetPosition(tiltMotor.getCurrentPosition() + tiltStep);
//            tiltMotor.setPower(tiltPower);
//        }
        //REQUEST
        if(gamepad2.y) {
            isManual = false;
            if(currTiltState == TiltState.INTAKE) {
                reqExtendState = ExtendState.INTAKE_EXTENDED;
            } else if(currTiltState == TiltState.OUTTAKE) {
                reqExtendState = ExtendState.EXTENDED;
            }
        }
        else if(gamepad2.a) {
            isManual = false;
            reqExtendState = ExtendState.SLIGHTLY_EXTENDED;
        }
        else if(gamepad2.x) {
            isManual = false;
            reqExtendState = ExtendState.SLIGHTLY_EXTENDED;
            reqTiltState = TiltState.INTAKE;
        }
        else if(gamepad2.b) {
            isManual = false;
            reqExtendState = ExtendState.SLIGHTLY_EXTENDED;
            reqTiltState = TiltState.OUTTAKE;
        }
        //ACTUATE
        if(!isManual) {
        if(reqExtendState == ExtendState.EXTENDED && currTiltState == TiltState.OUTTAKE) {
            slidesMotor.setTargetPosition(extendPos);
            slidesMotor.setPower(slidesPower);
            pivotServo.setPosition(pivotOuttake);
        } else if(reqExtendState == ExtendState.INTAKE_EXTENDED && currTiltState == TiltState.INTAKE) {
            slidesMotor.setTargetPosition(intakeExtendPos);
            slidesMotor.setPower(slidesPower);
        } else if(reqExtendState == ExtendState.SLIGHTLY_EXTENDED) {
            slidesMotor.setTargetPosition(slightlyExtendedPos);
            slidesMotor.setPower(slidesPower);
        }

        if(reqTiltState == TiltState.INTAKE && currExtendState == ExtendState.SLIGHTLY_EXTENDED) {
            tiltMotor.setTargetPosition(intakeTiltPos);
            tiltMotor.setVelocity(90, AngleUnit.DEGREES);
        } else if(reqTiltState == TiltState.OUTTAKE && currExtendState == ExtendState.SLIGHTLY_EXTENDED) {
            tiltMotor.setTargetPosition(outtakeTiltPos);
            tiltMotor.setPower(tiltPower);
            pivotServo.setPosition(pivotOuttake);
        }
        }
        //UPDATE
        if(tiltMotor.getCurrentPosition() > intakeTiltPos - 50) {
            currTiltState = TiltState.INTAKE;
        } else if(tiltMotor.getCurrentPosition() < outtakeTiltPos + 50) {
            currTiltState = TiltState.OUTTAKE;
            lockingServo.setPosition(open);
        } else {
            currTiltState = TiltState.IN_TRANSIT;
        }

        if(slidesMotor.getCurrentPosition() < slightlyExtendedPos - 50) {
            currExtendState = ExtendState.RETRACTED;
        } else if(slidesMotor.getCurrentPosition() > slightlyExtendedPos + 50) {
            currExtendState = ExtendState.EXTENDED;
        } else {
            currExtendState = ExtendState.SLIGHTLY_EXTENDED;
        }
    }

    public enum TiltState {
        IN_TRANSIT,
        INTAKE,
        OUTTAKE
    }

    public enum ExtendState {
        RETRACTED,
        SLIGHTLY_EXTENDED,
        INTAKE_EXTENDED,
        EXTENDED
    }
}
