package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;

@TeleOp(name="Drive", group="Linear Opmode")

public class Teleop extends LinearOpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor intakeL = null;
    private DcMotor intakeR = null;
    //private DcMotor arm1 = null;
    //private DcMotor arm2 = null;
    //private Servo clawR = null;
    //private Servo clawL = null;
    BNO055IMU imu;
    Orientation angles;
    //stoneTrack = new StoneTracker();
    //StoneTracker stoneTrack;
    double drive;
    double turn;
    double strafe;
    double leftback;
    double rightfront;
    double leftfront;
    double rightback;
    boolean rightStickStrafe = true;
    
    public void hardwareInit() {
        leftFront  = hardwareMap.get(DcMotor.class, "purple");
        rightFront = hardwareMap.get(DcMotor.class, "black");
        leftBack = hardwareMap.get(DcMotor.class, "red");
        rightBack = hardwareMap.get(DcMotor.class, "orange");
        //intakeL = hardwareMap.get(DcMotor.class, "green");
        //intakeR = hardwareMap.get(DcMotor.class, "blue");
        //arm1 = hardwareMap.get(DcMotor.class, "yellow");
        //arm2 = hardwareMap.get(DcMotor.class, "white");
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        //intakeL.setDirection(DcMotor.Direction.FORWARD);
        //intakeR.setDirection(DcMotor.Direction.REVERSE);
        //clawR.setDirection(Servo.Direction.REVERSE);
        //clawL.setDirection(Servo.Direction.FORWARD);
        //arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }
    //Run Opmode
    @Override
    public void runOpMode() {
        hardwareInit();
        //stoneTrack.init(hardwareMap);
        //stoneTrack.activate();
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            findZero(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle), 0.5);
            double claw = gamepad2.right_trigger - gamepad2.left_trigger;
            stonetrack();
            arm();
            pickBlock();
            intake();
            drive();
            powerToMotors();
            runTelemetry();
        }
    }

    public void runTelemetry() {
        //Show the elapsed game time, wheel power, and heading.
        telemetry.addData("Run Time:", runtime.toString());
        telemetry.addData("Motors:","RB:(%.2f) LB:(%.2f) RF:(%.2f) LF:(%.2f) D:(%.2f) T:(%.2f) S:(%.2f)", rightback,leftback,rightfront,leftfront,drive,turn,strafe);
        telemetry.addData("Heading: ", "(%.2f)", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        telemetry.update();
    }
    public void stonetrack() {
        /*if (stoneTrack.scan() > 0 && gamepad1.left_bumper) {
            double turn = stoneTrack.stone.estimateAngleToObject(AngleUnit.DEGREES) * 0.05;
            double drive = (300 - stoneTrack.stone.getHeight()) * 0.05;
        }
        telemetry.addData("(%.2f)", stoneTrack.toString());*/
    }
    public void powerToMotors() {
        //Send calculated power to motors
        leftfront = drive + turn + strafe;
        rightfront = drive - turn - strafe;
        leftback = drive + turn - strafe;
        rightback = drive - turn + strafe;
        leftBack.setPower(leftback);
        rightFront.setPower(rightfront);
        leftFront.setPower(leftfront);
        rightBack.setPower(rightback);
        //intakeR.setPower(intake);
        //intakeL.setPower(intake);
        //clawR.setPosition(claw);
        //clawL.setPosition(claw);
        //Run telemetry
    }
    public void pickBlock() {
        if (gamepad2.a) {
            //Grab Block Automaticlly (Needs Encoders)
        }
    }
    public void intake() {
        double intake;
        if(gamepad2.right_bumper || gamepad1.right_bumper) {
            for(double i = 5000; i <= 5000 && i > 2500;i -= Math.pow(i, 0.0035)) {
                intake = i/5000;
                sleep((10*1000)/2430);
            }
        }
        intake = 0;
    }
    public void findZero(double degrees, double error) {
        if (gamepad1.x) {
            double speed = 0.04;
            if (OnePressZero) {
                while (degrees <= -error || degrees >= error) {
                    if (degrees > 0) {
                        turn = degrees * speed + 0.5;
                    } else {
                        turn = degrees * speed - 0.5;
                    }
                    telemetry.addData("Turning:","(%.2f)", degrees * 0.06);
                }
            } else {
                if (degrees <= -error || degrees >= error) {
                    if (degrees > 0) {
                        turn = degrees * speed + 0.5;
                    } else {
                        turn = degrees * speed - 0.5;
                    }
                    telemetry.addData("Turning:","(%.2f)", degrees * 0.06);
                }
            }
        }
    }
    public void drive() {
        if(rightStickStrafe) {
            //Drive Normally
            if (!gamepad1.left_bumper && !gamepad1.x) {
                drive = -gamepad1.left_stick_y;
                strafe  = gamepad1.right_stick_x;
                turn = gamepad1.left_stick_x;
            }
            //Gamepad2 take control
            if (gamepad2.x && gamepad1.atRest() && !gamepad1.left_bumper) {
                drive = -gamepad2.left_stick_y*0.25;
                strafe = gamepad2.right_stick_x*0.25;
                turn = gamepad2.left_stick_x*0.25;
            }
            //Slow with y
            if (gamepad1.y) {
                drive = -gamepad1.left_stick_y*0.25;
                strafe  = gamepad1.right_stick_x*0.25;
                turn = gamepad1.left_stick_x*0.25;
            }
        } else {
            //Drive Normally
            if (!gamepad1.left_bumper && !gamepad1.x) {
                drive = -gamepad1.left_stick_y;
                strafe  = gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x;
            }
            //Gamepad2 take control
            if (gamepad2.x && gamepad1.atRest() && !gamepad1.left_bumper) {
                drive = -gamepad2.left_stick_y*0.25;
                strafe = gamepad2.left_stick_x*0.25;
                turn = gamepad2.right_stick_x*0.25;
            }
            //Slow with y
            if (gamepad1.y) {
                drive = -gamepad1.left_stick_y*0.25;
                strafe  = gamepad1.left_stick_x*0.25;
                turn = gamepad1.right_stick_x*0.25;
            }
        }
    }   
}
