package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class Flywheel {

    private DcMotorEx motorFlywheel;
    private DcMotor motorFlywheel2;


    public void init(HardwareMap hwMap) {
        motorFlywheel = hwMap.get(DcMotorEx.class, "motorFlywheel");
        motorFlywheel2 = hwMap.get(DcMotor.class, "motorFlywheel2");

        motorFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFlywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(200, 0, 0, 12.3);
        motorFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void flywheel(double targVel, boolean autoFlywheel){


        if(autoFlywheel){
            motorFlywheel.setVelocity(targVel);
            motorFlywheel2.setPower(motorFlywheel.getPower());
        } else {
            motorFlywheel.setPower(0);
            motorFlywheel2.setPower(0);
        }

    }
}
