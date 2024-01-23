package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DriveTenSeconds") //This is an annotation that makes this code appear in
// the autonomous area on the phone
// @Disable // Hides class from the controller interface
public class Drive10Seconds extends OpMode {
    //Declaring DC Motors
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotor[] motorsArray = {motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};
    double robotPower = 0.5;
    ElapsedTime timer = new ElapsedTime();
    double currentTime;

    @Override //needed for sdk, don't use this for your own functions
    public void init() {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void start(){
        timer.startTime();
    }

    @Override
    public void loop() {
        currentTime = timer.time();
        if(currentTime <= 10){
            for(DcMotor motor: motorsArray){
                motor.setPower(robotPower);
            }
        } else {
        for(DcMotor motor: motorsArray){
            motor.setPower(0);
        }}

    }

}
