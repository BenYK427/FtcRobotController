package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MechanumDrive {

    private DcMotor RightFront, RightBack, LeftFront, LeftBack;

    private IMU imu;

    public void init(HardwareMap hwMap){
        RightFront = hwMap.get(DcMotor.class, "RightFront");
        RightBack = hwMap.get(DcMotor.class, "RightBack");
        LeftFront = hwMap.get(DcMotor.class, "LeftFront");
        LeftBack = hwMap.get(DcMotor.class, "LeftBack");

        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public void drive(double forward, double strafe, double rotate){
        double RightFrontPower = forward - strafe - rotate;
        double RightBackPower = forward + strafe + rotate;
        double LeftFrontPower = forward + strafe + rotate;
        double LeftBackPower = forward - strafe + rotate;

        double maxPower = 1;
    }

}
