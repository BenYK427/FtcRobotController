package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    //private DcMotor turretMotor;

//    private double PID = 0;
//    private double i = 0;
//    private double d = 0;
//    private double error;
//    private double lastError;
//    private double targetPos;

    //public double turretPos;


    public void init(HardwareMap hwMap){
//        turretMotor = hwMap.get(DcMotor.class, "turretMotor");
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void turret(double turretPower){

//        targetPos = 400;
//
//        error = turretMotor.getCurrentPosition() - targetPos;
//        i = i + error;
//        d = error - lastError;
//        lastError = error;
//        //p:0.45 i:0.022 d:0.5
//
//            PID = error * 0.5 + i * 0.05 + d * 0;
//            turretPower = PID*0.001 + 0.1;
//
//        if(turretPower > 0.4){
//            turretPower = 0.4;
//        }
//
//
//            turretMotor.setPower( -turretPower);
    }
}