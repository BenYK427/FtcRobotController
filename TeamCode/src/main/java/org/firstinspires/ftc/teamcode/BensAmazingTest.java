package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MechanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

@TeleOp
public class BensAmazingTest extends OpMode{

    MechanumDrive drive = new MechanumDrive();
    double forward, strafe, turn, headingDrive;
    boolean adjust = false;

    Intake intake = new Intake();
    double intakePower;

    Flywheel flywheel = new Flywheel();
    double targVel = 0;
    boolean autoFlywheel;

    //Turret turret = new Turret();
    double turretPower;

    //private DcMotor turretMotor;
    GoBildaPinpointDriver odo;

    private Limelight3A limelight;

    double tx;

    private Servo triggerServo;


    private DcMotor turretMotor;
    double i;
    double d;
    double error;
    double lastError;
    double PID;
    double txpmo;
    double targetPos = 0;
    double lllocalize = 0;
    double turretAnchor = 0;
    double zeroAdjust = 0;
    double headingNormShift = 0;
    double headingNormShiftVal = 0;
    boolean adjustedTur = false;


    double intakePowerShoot = 0;


    public void init(){
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        flywheel.init(hardwareMap);
        //turret.init(hardwareMap);

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        triggerServo = hardwareMap.get(Servo.class, "triggerServo");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 100, 100, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addLine("init complete");
        adjustedTur = false;

    }

    public void start(){
        limelight.start();
        adjustedTur = false;
    }

    @Override
    public void loop() {
        //-----------------------Odometry---------------------------
        double heading = odo.getHeading(UnnormalizedAngleUnit.RADIANS);
        double headingNorm = odo.getHeading(AngleUnit.RADIANS)*800/3.14159;

        double x = odo.getPosX(DistanceUnit.MM);

        //----------------------------Limelight-------------------------
        limelight.updateRobotOrientation(headingNorm);
        LLResult llResult = limelight.getLatestResult();

        Pose3D botPose = llResult.getBotpose();

        tx = llResult.getTx()*5.4166667;



        if(headingNormShiftVal > 0) {
            if (headingNorm + headingNormShiftVal > 800) {
                headingNormShift = headingNorm - (1600 - headingNormShiftVal);
            } else {
                headingNormShift = headingNorm + headingNormShiftVal;
            }
        }
        if(headingNormShiftVal < 0) {
            if (headingNorm + headingNormShiftVal < -800) {
                headingNormShift = headingNorm + 1600 + headingNormShiftVal;
            } else {
                headingNormShift = headingNorm + headingNormShiftVal;
            }
        }

        if(adjustedTur == false){
            headingNormShift = headingNorm;
        }



        //---------------------------Drive------------------------------
        if(gamepad1.b){
            adjust = true;
        } else {
            adjust = false;
        }

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x*-1.1;
        turn = gamepad1.right_stick_x*-0.4;

        headingDrive = odo.getHeading(AngleUnit.RADIANS);

        drive.drive(forward, strafe, turn, adjust, headingDrive);

        //-----------------------Shooting-----------------------
        if(gamepad2.y){
            triggerServo.setPosition(0.3);
        } else {
            triggerServo.setPosition(0);
        }



        if(triggerServo.getPosition() > 0.25 && gamepad2.left_stick_y == 0){
            intakePowerShoot = -1;
        } else {
            intakePowerShoot = 0;
        }

        //-----------------------Intake-------------------------
        intakePower = gamepad2.left_stick_y + intakePowerShoot;

        intake.intake(intakePower);

        //-----------------------Turret-------------------------
        if(gamepad2.dpadUpWasPressed()){
            zeroAdjust += 10;
        }
        if(gamepad2.dpadDownWasPressed()){
            zeroAdjust -= 10;
        }

        if(tx != 0){

            if(Math.abs(error + tx ) > 30 && targetPos != 0 && gamepad2.right_stick_x == 0 &&  Math.abs(error + tx) < 500 && autoFlywheel == true){
                lllocalize += error + tx;
            }


            if(Math.abs(tx) < 100 && gamepad2.right_stick_x != 0){
                turretAnchor = turretMotor.getCurrentPosition();
            }

            if(gamepad2.a){
                turretAnchor = turretMotor.getCurrentPosition();
                lllocalize = 0;
            }
        }

        if(Math.abs(error) < 20 && adjustedTur == false && tx != 0){
            headingNormShiftVal = turretMotor.getCurrentPosition();
            adjustedTur = true;
            turretAnchor = 0;
        }

        if(autoFlywheel){
            targetPos = headingNormShift + turretAnchor + lllocalize + zeroAdjust;
        } else {
            targetPos = 0 + zeroAdjust;
        }
        //targetPos = headingNormShift + turretAnchor + lllocalize + zeroAdjust;

        error = turretMotor.getCurrentPosition() - targetPos;
        i = i + error;
        if(Math.abs(error) > 10){
            i = 0;
        }
        d = error - lastError;
        lastError = error;
        //p:1 i:0.05 d:0.05

        PID = error * 1 + i * 0.05 + d * 0.05;
        PID = PID*0.005;

        if(PID > 0.5){
            PID = 0.5;
        }

        if(PID < -0.5){
            PID = -0.5;
        }

        if(Math.abs(error) < 80){
            if(PID > 0.2){
                PID = 0.2;
            }

            if(PID < -0.2){
                PID = -0.2;
            }
        }



        if(Math.abs(error) < 1){
            PID = 0;
        }

        if(gamepad2.right_stick_x != 0){
            PID = -gamepad2.right_stick_x*0.5;
        }


        turretMotor.setPower(-PID);

//        if(turretMotor.getCurrentPosition() < -750 && PID > 0){
//            turretMotor.setPower(0);
//        }

        //----------------------Flywheel------------------------
        targVel = llResult.getTa()*-1195.3 +2230;

        if(gamepad2.rightBumperWasPressed()){
            autoFlywheel = !autoFlywheel;
        }



        flywheel.flywheel(targVel, autoFlywheel);

        //--------------------Telemetry-------------------------

        telemetry.addData("heading", headingNorm);
        telemetry.addData("headingshift", headingNormShift);
        telemetry.addData("shiftval", headingNormShiftVal);
        telemetry.addData("adj", adjustedTur);
        telemetry.addData("distance", triggerServo.getPosition());
        telemetry.addData("tx", tx);
        telemetry.addData("ta", llResult.getTa());
        telemetry.addData("error", error);
        telemetry.addData("lllocalize", lllocalize);
        telemetry.addData("turret Power", turretMotor.getPower());
        telemetry.addData("targpos", targetPos);
        telemetry.addData("anchor", turretAnchor);
        telemetry.addData("turretPos", turretMotor.getCurrentPosition());
        telemetry.update();

        odo.update();
    }
}
