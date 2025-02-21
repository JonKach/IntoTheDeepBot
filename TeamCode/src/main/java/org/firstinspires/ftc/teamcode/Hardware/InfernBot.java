package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class InfernBot {
    public Chassis chassis;
    public Arm arm;
    public Slides slides;
    public Tilt tilt;
    public static boolean ranAuto = false;

    public InfernBot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.chassis = new Chassis(hardwareMap, gamepad1, telemetry);
        this.arm = new Arm(hardwareMap, gamepad2, telemetry);
//        this.slides = new Slides(hardwareMap, gamepad2);
//        this.tilt = new Tilt(hardwareMap, gamepad2);

    }
}
