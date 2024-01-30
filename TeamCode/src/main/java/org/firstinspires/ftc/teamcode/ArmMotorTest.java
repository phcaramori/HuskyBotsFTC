package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Arm Motor Test")
public class ArmMotorTest extends OpMode {

    DcMotor motorArm;
    double armPower = 0;
    @Override
    public void init() {
        motorArm = hardwareMap.get(DcMotor.class, "ArmMotor");
    }

    @Override
    public void loop() {
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPower = 0;
        if(gamepad1.dpad_up) armPower = 0.3;
        else if(gamepad1.dpad_down) armPower = -0.3;
        motorArm.setPower(armPower);
    }
}
