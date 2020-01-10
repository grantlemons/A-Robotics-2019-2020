package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Timer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;
import android.graphics.Color;

public class Drivetrain {
    //Create motor objects
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private BNO055IMU imu = null;
    private NormalizedColorSensor colorSensor = null;
    private Telemetry telemetry;
    double leftBackPwrCurrent;
    double leftFrontPwrCurrent;
    double rightBackPwrCurrent;
    double rightFrontPwrCurrent;
    ElapsedTime timer;
    double lastTime;

    public Drivetrain(HardwareMap hardwareMap, Telemetry aTelemetry) {
        //Assign vars to ports
        leftFront  = hardwareMap.get(DcMotor.class, "purple");
        leftBack = hardwareMap.get(DcMotor.class, "red");
        rightFront = hardwareMap.get(DcMotor.class, "orange");
        rightBack = hardwareMap.get(DcMotor.class, "black");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // imu setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        imu.initialize(parameters);
        
        // Color Sensor setup
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
          ((SwitchableLight)colorSensor).enableLight(true);
        }
        
        // keep access to telemetry
        telemetry = aTelemetry;
        
        //Timer init
        timer = new ElapsedTime();
        lastTime = 0;
    }
    
    public void driveWithLimit(double forward, double turn, double strafe){
        double elapsedTime = timer.milliseconds() - lastTime;
        //Calculate power
        double leftFrontPwrTarget = forward + turn + strafe;
        double rightFrontPwrTarget = forward - turn - strafe;
        double leftBackPwrTarget = forward + turn - strafe;
        double rightBackPwrTarget = forward - turn + strafe;
        
        leftFrontPwrCurrent = leftFront.getPower();
        leftBackPwrCurrent = leftBack.getPower();
        rightFrontPwrCurrent = rightFront.getPower();
        rightBackPwrCurrent = rightBack.getPower();
        
        //Power to motors
        leftFront.setPower(getNewPower(elapsedTime, leftFrontPwrCurrent, leftFrontPwrTarget));
        leftBack.setPower(getNewPower(elapsedTime, leftBackPwrCurrent, leftBackPwrTarget));
        rightFront.setPower(getNewPower(elapsedTime, rightFrontPwrCurrent, rightFrontPwrTarget));
        rightBack.setPower(getNewPower(elapsedTime, rightBackPwrCurrent, rightBackPwrTarget));
        lastTime = timer.milliseconds();
    }
    
    private double getNewPower(double elapsedTime, double currentPower, double targetPower) {
        double maxAccel = 1.0/150;
        double powerChange = maxAccel*elapsedTime;
        
        if (targetPower > currentPower) {
            return Math.min(currentPower+powerChange, targetPower);
        }
        if (targetPower < currentPower) {
            return Math.max(currentPower-powerChange, targetPower);
        }
        return currentPower;
    }
    
    public void drive(double forward, double turn, double strafe) {
        //Calculate power
        double leftFrontPwr = forward + turn + strafe;
        double rightFrontPwr = forward - turn - strafe;
        double leftbBackPwr = forward + turn - strafe;
        double rightBackPwr = forward - turn + strafe;
        //Power to motors
        leftFront.setPower(leftFrontPwr);
        leftBack.setPower(leftbBackPwr);
        rightFront.setPower(rightFrontPwr);
        rightBack.setPower(rightBackPwr);
        
/**********
NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("H", "%.3f", hsvValues[0])
                .addData("S", "%.3f", hsvValues[1])
                .addData("V", "%.3f", hsvValues[2]);
**********/
    }
    
    public boolean setHeading(double targetHeading) {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        double speed = 0.02;
        double error = 0.5;
        double currentHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        double turnDegrees = currentHeading - targetHeading;
        if(turnDegrees < -180) {
            turnDegrees += 360;
        }
        if(turnDegrees > 180) {
            turnDegrees -= 360; 
        }
        if(Math.abs(turnDegrees) < error) {
            drive(0, 0, 0);
            return true;
        }
        else if (turnDegrees > 0) {
            drive(0, turnDegrees * speed + 0.05, 0);
        } else {
            drive(0, turnDegrees * speed - 0.05, 0);
        }
        telemetry.addData("Turning:","(%.2f)", currentHeading * speed);
        return false;
    }
    
    public double headingAdjust(double targetHeading) {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        double speed = 0.02;
        double currentHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        double turnDegrees = currentHeading - targetHeading;

        if(turnDegrees < -180) {
            turnDegrees += 360;
        }
        if(turnDegrees > 180) {
            turnDegrees -= 360; 
        }

        if (turnDegrees > 1) {
            return turnDegrees * speed + 0.05;
        } else if (turnDegrees < 1) {
            return turnDegrees * speed - 0.05;
        } else {
            return 0;
        }
    }

    public void forwardDistance(int moveDistance, double power) {

        // Conversion is 4000 counts per 33.2 inches
        int deltaPosition = moveDistance * 40000 / 332;
        
        int targetLeftFrontPos = leftFront.getCurrentPosition() + deltaPosition;
        int targetLeftBackPos = leftBack.getCurrentPosition() + deltaPosition;
        int targetRightFrontPos = rightFront.getCurrentPosition() + deltaPosition;
        int targetRightBackPos = rightBack.getCurrentPosition() + deltaPosition;

        leftFront.setTargetPosition(targetLeftFrontPos);
        leftBack.setTargetPosition(targetLeftBackPos);
        rightFront.setTargetPosition(targetRightFrontPos);
        rightBack.setTargetPosition(targetRightBackPos);
        
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(power, 0, 0);

        //try {Thread.sleep(5000);} catch(InterruptedException ule) {}

        while(Math.abs(leftFront.getCurrentPosition() - targetLeftFrontPos) > 5) {}
        while(Math.abs(leftBack.getCurrentPosition() - targetLeftBackPos) > 5) {}
        while(Math.abs(rightFront.getCurrentPosition() - targetRightFrontPos) > 5) {}
        while(Math.abs(rightBack.getCurrentPosition() - targetRightBackPos) > 5) {}

        drive(0, 0, 0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeDistance(int moveDistance, double power) {

        // Conversion is 4000 counts per 33.2 inches
        int deltaPosition = moveDistance * 40000 / 332;
        
        int targetLeftFrontPos = leftFront.getCurrentPosition() + deltaPosition;
        int targetLeftBackPos = leftBack.getCurrentPosition() - deltaPosition;
        int targetRightFrontPos = rightFront.getCurrentPosition() - deltaPosition;
        int targetRightBackPos = rightBack.getCurrentPosition() + deltaPosition;

        leftFront.setTargetPosition(targetLeftFrontPos);
        leftBack.setTargetPosition(targetLeftBackPos);
        rightFront.setTargetPosition(targetRightFrontPos);
        rightBack.setTargetPosition(targetRightBackPos);
        
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(power, 0, 0);

        //try {Thread.sleep(5000);} catch(InterruptedException ule) {}

        while(Math.abs(leftFront.getCurrentPosition() - targetLeftFrontPos) > 5) {}
        while(Math.abs(leftBack.getCurrentPosition() - targetLeftBackPos) > 5) {}
        while(Math.abs(rightFront.getCurrentPosition() - targetRightFrontPos) > 5) {}
        while(Math.abs(rightBack.getCurrentPosition() - targetRightBackPos) > 5) {}

        drive(0, 0, 0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void forwardToColorNoStop(String searchColor, double power, double heading) {

        // Priming read
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Allow for some variance in hue and saturation
        while ((searchColor.equals("blue") && hsvValues[0] < 180) || 
                (searchColor.equals("red") && hsvValues[0] > 0) ||
                hsvValues[1] < 0.7) {
            drive(power, headingAdjust(heading), 0);
            colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
        }
    }
}
