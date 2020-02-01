package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
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
import java.util.List;
import java.util.Locale;

public class Slide {
    //Create motor objects
    private DcMotor extendMotor = null;
    int min = 0;
    int max = 3000;
    int bufferDistance = 200;
    double speed = 1;
    
    public Slide(HardwareMap hardwareMap) {
        extendMotor = hardwareMap.get(DcMotor.class, "green");
        //extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setDirection(DcMotor.Direction.FORWARD);
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveToPosition(int targetPos) {
        extendMotor.setTargetPosition(targetPos);
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(1);
        while(Math.abs(extendMotor.getCurrentPosition() - targetPos) > 5) {}
        try {Thread.sleep(100);} catch(InterruptedException ule) {}
        extendMotor.setPower(0);
    }
    
    public void fullyExtend() {
        moveToPosition(max);
    }
    
    public void fullyRetract() {
        moveToPosition(min);
    }
    
    public void calibrate() {
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveToPosition(-3100);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double extend(double input) {
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendMotor.setPower(input);
        return extendMotor.getCurrentPosition();
    }
    
    public void extendRestricted(double input) {
        if(input < 0) {
            if(extendMotor.getCurrentPosition() >= min) {
                extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(extendMotor.getCurrentPosition()-min < bufferDistance) {
                    extendMotor.setPower(input * ((extendMotor.getCurrentPosition()-min)/bufferDistance)-0.05);
                } else {
                    extendMotor.setPower(input*speed);
                }
            } else {
                extendMotor.setPower(0);
            }
        } else if(input > 0) {
            if(extendMotor.getCurrentPosition() <= max) {
                extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(max-extendMotor.getCurrentPosition() < bufferDistance) {
                    extendMotor.setPower(input * ((max-extendMotor.getCurrentPosition())/bufferDistance)+0.05);
                } else {
                    extendMotor.setPower(input*speed);
                }
            } else {
                extendMotor.setPower(0);
            }
        } else {
            extendMotor.setPower(0);
        }
    }
}
