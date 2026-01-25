package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MechanumDrive {
    private DcMotor RightFront, RightBack, LeftFront, LeftBack;

    private IMU imu;

    private double adjustment = 0;

    public void init(HardwareMap hwMap){
        RightFront = hwMap.get(DcMotor.class, "RightFront");
        RightBack = hwMap.get(DcMotor.class, "RightBack");
        LeftFront = hwMap.get(DcMotor.class, "LeftFront");
        LeftBack = hwMap.get(DcMotor.class, "LeftBack");

        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }






    public void drive(double forward, double strafe, double turn, boolean adjust, double headingDrive){

        double r = Math.sqrt(strafe*strafe+forward*forward);
        double theda = Math.atan2(forward, strafe);
        double headingPositive;
        //adjustment mid match:
        if(adjust){
            adjustment = headingDrive;
        }
        //apply rotation:
        if(headingDrive < 0){
            headingPositive = headingDrive+2*3.14159;
        } else {
            headingPositive = headingDrive;
        }
        theda = theda - headingPositive + adjustment;
        //convert back to cartisain:
        double newforward = r*Math.sin(theda);
        double newstrafe = r*Math.cos(theda);
        double denominator = Math.max(Math.abs(newforward) + Math.abs(newstrafe) + Math.abs(turn), 1);

        RightFront.setPower((newforward - newstrafe - turn) / denominator);
        RightBack.setPower((newforward + newstrafe - turn) / denominator);
        LeftFront.setPower((newforward - newstrafe + turn) / denominator);
        LeftBack.setPower((newforward + newstrafe + turn) / denominator);
    }


}
