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

public class Elbow {
    private DcMotor elbowMotor = null;
    int min = -1300;
    int level = 0;
    int max = 1500;
    int[] blockPositions = {1000, 550, 0, -700};
    int currentBlockLevel = -1;
    
    public Elbow(HardwareMap hardwareMap) {
        elbowMotor = hardwareMap.get(DcMotor.class, "blue");
        //elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setDirection(DcMotor.Direction.FORWARD);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveToPosition(int targetPos) {
        elbowMotor.setTargetPosition(targetPos);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setPower(0.8);
        while(Math.abs(elbowMotor.getCurrentPosition() - targetPos) > 5) {}
        try {Thread.sleep(100);} catch(InterruptedException ule) {}
        elbowMotor.setPower(0);
    }
    
    public void fullyDown() {
        currentBlockLevel = -1; // reset
        moveToPosition(max);
    }
    
    public void almostFullyDown() {
        currentBlockLevel = -1; // reset
        moveToPosition(max - 150);
    }
    
    public void toZero() {
        currentBlockLevel = 2; // reset to reflect first block position
        moveToPosition(0);
    }
    
    public void toDrivingPos() {
        currentBlockLevel = 0; // reset
        moveToPosition(blockPositions[currentBlockLevel]);
    }
    
    public void calibrate() {
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveToPosition(-1500);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public double move(double input) {
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setPower(input*0.25);
        return elbowMotor.getCurrentPosition();
    }
    
    public void moveRestricted(double input) {
        int bufferDistance = 200;
        double speed = 0.75;
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(input < 0) {
            if(elbowMotor.getCurrentPosition() >= min) {
                elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(elbowMotor.getCurrentPosition()-min < bufferDistance) {
                    elbowMotor.setPower(input * ((elbowMotor.getCurrentPosition()-min)/bufferDistance)-0.05);
                } else {
                    elbowMotor.setPower(input*speed);
                }
            } else {
                elbowMotor.setPower(0);
            }
        } else if(input > 0) {
            if(elbowMotor.getCurrentPosition() <= max) {
                elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(max-elbowMotor.getCurrentPosition() < bufferDistance) {
                    elbowMotor.setPower(input * ((max-elbowMotor.getCurrentPosition())/bufferDistance)+0.05);
                } else {
                    elbowMotor.setPower(input*speed);
                }
            } else {
                elbowMotor.setPower(0);
            }
        } else {
            elbowMotor.setPower(0);
        }
    }
}
