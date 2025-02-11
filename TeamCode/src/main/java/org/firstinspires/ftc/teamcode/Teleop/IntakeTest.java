package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp
public class IntakeTest extends OpMode {
CRServo leftServo;
CRServo rightServo;

    @Override
    public void init() {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            leftServo.setPower(-1);
            rightServo.setPower(1);
        } else {
            leftServo.setPower(0);
            rightServo.setPower(0);
        }
    }
}
