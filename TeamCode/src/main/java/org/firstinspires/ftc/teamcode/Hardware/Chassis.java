package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Chassis {
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
//    public DcMotorEx leftOdo, strafeOdo, rightOdo;

    Gamepad gamepad1;
    public static IMU imu; //A static IMU object that shares heading between TeleOp and Auton
    public double headingOffset = 180; //OPPOSITE //stores the heading the robot started the opmode with (corrects for error)
    double driveSpeed = 1.0;

    Telemetry telemetry;

    //TURN PID
    double integralSum_turn = 0;
    public static double kP_turn = 0.019;
    public static double kI_turn = 0.000001;
    public static double kD_turn = 0;

    ElapsedTime timer_turn = new ElapsedTime();
    double lastError_turn = 0;

    //DRIVE PID
    double integralSum_drive = 0;
    public static double kP_drive = 0.001;
    public static double kI_drive = 0.0004;
    public static double kD_drive;

    public static double driveTicksPerIn = 44.5; //CHANGE IF NEEDED
    public int avgCurrDriveTicks = 0;

    ElapsedTime timer_drive = new ElapsedTime();
    double lastError_drive = 0;

    //Teleop Constructor
    public Chassis(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry){
        this.gamepad1 = gamepad1;

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        backRightMotor = hardwareMap.get(DcMotor.class, "BRM");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        this.telemetry = telemetry;
    }

    public void fieldCentricDrive(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if(gamepad1.right_stick_button) { //resets field-centric drive heading (offset = current heading)
            headingOffset = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        double botHeading = Math.toRadians(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - headingOffset);

//        double botHeading = 0;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

//        telemetry.addData("denominator", denominator);
//        telemetry.addData("rotY", rotY);
//        telemetry.addData("rotX", rotX);
//        telemetry.addData("rx", rx);
//        telemetry.addData("frontLeft", frontLeftPower);
//        telemetry.addData("backLeft", backLeftPower);
//        telemetry.addData("frontRight", frontRightPower);
//        telemetry.addData("backRight", backRightPower);

        if(gamepad1.right_bumper) //slow down button for fine-control
            driveSpeed = 0.3;
        else
            driveSpeed = 1;

        frontLeftMotor.setPower(frontLeftPower * driveSpeed);
        backLeftMotor.setPower(backLeftPower * driveSpeed);
        frontRightMotor.setPower(frontRightPower * driveSpeed);
        backRightMotor.setPower(backRightPower * driveSpeed);
    }

    public void imuTelemetry(Telemetry telemetry) {
        telemetry.addData("imuHeading", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("botHeading", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - headingOffset);
        telemetry.addData("currDriveTicks", (frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4);
    }

    public void autoTurn(double angleToTurnTo) {
        turnPID(angleToTurnTo, -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - headingOffset);
    }

    public void autoDrive(double ticksToDriveTo) {
        avgCurrDriveTicks = (frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4;
        telemetry.addData("currTicks", avgCurrDriveTicks);
        drivePID(ticksToDriveTo, avgCurrDriveTicks);
    }

    public void turnPID(double ref, double state) {
        double error = angleWrap(ref-state);
        integralSum_turn += error * timer_turn.seconds();
        double derivative = (error - lastError_turn) / timer_turn.seconds();
        lastError_turn = error;

        timer_turn.reset();

        double power = (error * kP_turn) + (derivative * kD_turn) + (integralSum_turn * kI_turn);
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
    }

    private double angleWrap(double degrees) {
        while(degrees > 180) {
            degrees -= 360;
        }
        while(degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }

    public void drivePID(double ref, double state) {
        double error = ref-state;
        integralSum_drive += error * timer_drive.seconds();
        telemetry.addData("error", error);
        telemetry.addData("integral sum", integralSum_drive);
        double derivative = (error - lastError_drive) / timer_drive.seconds();
        lastError_drive = error;

        timer_drive.reset();


        double power = (error * kP_drive) + (derivative * kD_drive) + (integralSum_drive * kI_drive);
        telemetry.addData("pwr", power);
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void stopDrive() {
        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }

    public void resetDriveTimer() {
        timer_drive.reset();
    }

    public void resetTurnTimer() {
        timer_turn.reset();
    }
}
