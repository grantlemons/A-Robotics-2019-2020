{
    // Declare OpMode members.
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
    //StoneTracker
    //stoneTrack = new StoneTracker();
    //StoneTracker stoneTrack;
    double drive;
    double turn;
    double strafe;
    double leftback;
    double rightfront;
    double leftfront;
    double rightback;
    
    //Define Hardware
    public void hardwareInit() {
        //Hardware Map
        leftFront  = hardwareMap.get(DcMotor.class, "purple");
        rightFront = hardwareMap.get(DcMotor.class, "black");
        leftBack = hardwareMap.get(DcMotor.class, "red");
        rightBack = hardwareMap.get(DcMotor.class, "orange");
        //intakeL = hardwareMap.get(DcMotor.class, "green");
        //intakeR = hardwareMap.get(DcMotor.class, "blue");
        //arm1 = hardwareMap.get(DcMotor.class, "yellow");
        //arm2 = hardwareMap.get(DcMotor.class, "white");
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        //Direction Setting
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
    }
    //Turn until degrees is ~0
    public void findZero(double degrees, double error) {
        if (degrees <= -error || degrees >= error) {
            telemetry.addData("Status:","Turning");
            turn = degrees * 0.06;
            //telemetry.addData("Status:"+"RB:(%.2f) LB:(%.2f) RF:(%.2f) LF:(%.2f)", rightback, leftback, rightfront, leftfront);
        }
    }
    //Run Opmode
    @Override
    public void runOpMode() {
        hardwareInit();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        //stoneTrack.init(hardwareMap);
        //stoneTrack.activate();
        /*if (stoneTrack.scan() > 0 && gamepad1.left_bumper) {
            double turn = stoneTrack.stone.estimateAngleToObject(AngleUnit.DEGREES) * 0.05;
            double drive = (300 - stoneTrack.stone.getHeight()) * 0.05;
        }*/
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            //defines heading
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //Self right when x is pressed
            /*if (gamepad1.x){
                telemetry.addData("Status:","Read Button");
                //findZero(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle), 4);
            }*/
            //When y is pressed: slow down
            /*if (gamepad1.y) {
                //Fine Ajustment
                drive = -gamepad1.left_stick_y*0.25;
                strafe  = gamepad1.left_stick_x*0.25;
                turn = gamepad1.right_stick_x*0.25;
            } */
            //Unless stoneTrack or findZero, drive normally
            if (!gamepad1.left_bumper && !gamepad1.x) {
                //Inputs
                drive = -gamepad1.left_stick_y;
                strafe  = gamepad1.right_stick_x;
                turn = gamepad1.left_stick_x;
            }

        //Gamepad2
            //Intake
            double intake;
            if(gamepad2.right_bumper || gamepad1.right_bumper) {
                for(double i = 5000; i <= 5000 && i > 2500;i -= Math.pow(i, 0.0035)) {
                    intake = i/5000;
                    sleep((10*1000)/2430);
                }
            }
            intake = 0;
            if (gamepad2.a) {
                //Grab Block Automaticlly (Needs Encoders)
                //pickBlock();
            }
            //Take control (Small Ajustments)
            if (gamepad2.x && gamepad1.atRest() && !gamepad1.left_bumper) {
                drive = -gamepad2.left_stick_y*0.25;
                strafe = gamepad2.left_stick_x*0.25;
                turn = gamepad2.right_stick_x*0.25;
            }   
            //Claw movement
            //double claw = gamepad1.right_trigger - gamepad1.left_trigger;
            //Unless Y is pressed, move arm
            if (!gamepad2.x) {
                double[] armRange = new double[]{0,1440/360*180};
                //Move Arm
                /*if (arm1.getCurrentPosition() >= armRange[1]-500) {
                    arm2.setTargetPosition(Range.clip(-gamepad2.left_stick_y*armRange[1], armRange[0],armRange[1]));
                } else {
                    arm1.setTargetPosition(Range.clip(-gamepad2.left_stick_y*armRange[1], armRange[0],armRange[1]));
                }*/
            }
            // Send calculated power to motors
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
            runTelemetry();
        }
    }

    public void runTelemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Run Time:", runtime.toString());
        telemetry.addData("Motors:","RB:(%.2f) LB:(%.2f) RF:(%.2f) LF:(%.2f) D:(%.2f) T:(%.2f) S:(%.2f)", rightback,leftback,rightfront,leftfront,drive,turn,strafe);
        telemetry.addData("Heading: ", "(%.2f)",AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        //telemetry.addData("(%.2f)", stoneTrack.toString());
        telemetry.update();
    }
    public void pickBlock() {
        //arm moves down to pick up
        //closes on block
    }
}
