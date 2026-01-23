package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    private DcMotor turretMotor;

    private double PID = 0;
    private double i = 0;
    private double d = 0;
    private double error;
    private double lastError;


    public void init(HardwareMap hwMap){
        turretMotor = hwMap.get(DcMotor.class, "turretMotor");

//        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void turret(double PID, double turretPower){

//        i = i + txpmo;
//        d = txpmo - lastError;
//        lastError = txpmo;
//        //p:0.45 i:0.022 d:0.5
//
//            PID = txpmo * 0.2 + i * 0 + d * 0;
//            PID = PID * 0.02;
//
//
//        if(txpmo == 0){
//            PID = 0;
//        }

            turretMotor.setPower(-PID + turretPower);
    }
}