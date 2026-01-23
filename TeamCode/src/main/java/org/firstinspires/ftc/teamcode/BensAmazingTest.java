package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
    double forward, strafe, turn;
    boolean adjust;

    Intake intake = new Intake();
    double intakePower;

    Flywheel flywheel = new Flywheel();
    double distance;
    boolean autoFlywheel;

    Turret turret = new Turret();

    double turretPower;
    GoBildaPinpointDriver odo;

    private Limelight3A limelight;


    //private DcMotor turretMotor;
    double i;
    double d;
    double lastError;
    double PID;
    double txpmo;


    public void init(){
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        flywheel.init(hardwareMap);
        turret.init(hardwareMap);

        //turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 100, 100, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addLine("init complete");

    }

    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        //-----------------------Odometry---------------------------
        double heading = odo.getHeading(UnnormalizedAngleUnit.RADIANS);
        double headingNorm = odo.getHeading(AngleUnit.RADIANS);

        double x = odo.getPosX(DistanceUnit.MM);

        //----------------------------Limelight-------------------------
        limelight.updateRobotOrientation(headingNorm);
        LLResult llResult = limelight.getLatestResult();

        Pose3D botPose = llResult.getBotpose();
        double tx = llResult.getTx();




        //---------------------------Drive------------------------------
        if(gamepad1.b){
            adjust = true;
        } else {
            adjust = false;
        }

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x*-1.1;
        turn = gamepad1.right_stick_x*-0.4;

        drive.drive(forward, strafe, turn, adjust);


        //-----------------------Intake-------------------------
        intakePower = gamepad2.left_stick_y;

        intake.intake(intakePower);

        //-----------------------Turret-------------------------
        if(gamepad2.b){
            txpmo = 0;
            turretPower = gamepad2.right_stick_x;
        }

        i = i + tx;
        d = tx - lastError;
        lastError = tx;
        //p:0.45 i:0.022 d:0.5

            PID = tx * 1 + i * 0.05 + d * 0;
            PID = PID * 0.05 + 0.1;


        if(tx == 0 && lastError == 0){
            PID = 0;
        }

        if(PID > 0.4){
            PID = 0.4;
        }


        turret.turret(PID, turretPower);
        //----------------------Flywheel------------------------
        distance = 200;

        if(gamepad2.rightBumperWasPressed()){
            autoFlywheel = !autoFlywheel;
        }



        flywheel.flywheel(distance, autoFlywheel);

        //--------------------Telemetry-------------------------

        telemetry.addData("heading", heading);
        telemetry.addData("x", x);
        //telemetry.addData("status", autoFlywheel);
        telemetry.addData("tx", tx);
        telemetry.addData("turret Power", PID);
        telemetry.update();

        odo.update();
    }
}
