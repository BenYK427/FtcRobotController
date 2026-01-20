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


    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }



    public void drive(double forward, double strafe, double turn, boolean adjust){


//        double frontLeftPower = forward + strafe + turn;
//        double backLeftPower = forward - strafe + turn;
//        double frontRightPower = forward - strafe - turn;
//        double backRightPower = forward + strafe - turn;
//
//        double maxPower = 1;
//        double maxSpeed = 1;
//
//        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//
//        LeftFront.setPower(maxSpeed*(frontLeftPower / maxPower));
//        LeftBack.setPower(maxSpeed*(backLeftPower / maxPower));
//        RightFront.setPower(maxSpeed*(frontRightPower / maxPower));
//        RightBack.setPower(maxSpeed*(backRightPower / maxPower));

        double r = Math.sqrt(strafe*strafe+forward*forward);
        double theda = Math.atan2(forward, strafe);
        double headingPositive;
        //adjustment mid match:
        if(adjust){
            adjustment = getHeading();
        }
        //apply rotation:
        if(getHeading() < 0){
            headingPositive = getHeading()+2*3.14159;
        } else {
            headingPositive = getHeading();
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
