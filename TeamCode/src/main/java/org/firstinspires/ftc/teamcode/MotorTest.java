package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Testing") //name of program in driver hub
public class MotorTest extends OpMode //class MUST EXTEND OpMode
{
    DcMotor motor1;
    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class,"motor1");
        //typeOfHardware.class, name set in driver hub config as string
    }

    /* Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY */
    @Override
    public void init_loop() {
    }

    /* Code to run ONCE when the driver hits PLAY */
    @Override
    public void start() {
    }

    /* Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP */
    @Override
    public void loop() {
        motor1.setPower(0.5); // from -1 to 1.
    }

    /* Code to run ONCE after the driver hits STOP */
    @Override
    public void stop() {
    }
}
