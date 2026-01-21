package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTest extends OpMode {
    private DcMotorEx motorFlywheel;
    private DcMotor motorFlywheel2;

    double P = 0;
    double F = 0;


    double[] stepSizes = {10, 1, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");
        motorFlywheel2 = hardwareMap.get(DcMotor.class, "motorFlywheel2");

        motorFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        motorFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex +1)% stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        motorFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        motorFlywheel.setVelocity(500);
        //motorFlywheel2.setPower(motorFlywheel.getPower());

        double curVel = motorFlywheel.getVelocity();
        double error = 500 - curVel;

        telemetry.addData("error", error);
        telemetry.addData("p", P);
        telemetry.addData("f", F);
        telemetry.addData("step size", stepSizes[stepIndex]);
    }
}
