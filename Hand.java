package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Hand {
    public Servo handServo = null;
    
    public Hand(HardwareMap hardwareMap) {
        handServo = hardwareMap.get(Servo.class, "servo0");
    }
    
    public void open() {
        handServo.setPosition(0);
    }
    
    public void close() {
        handServo.setPosition(0.55);
    }
    
    public void closeStart() {
        handServo.setPosition(0.7);
        
    }
}
