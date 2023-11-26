package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;




@TeleOp(name="TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2;

        // Variables
        double ClawOffset = 0; // offsets Claw starting position
        int StartingArmPosition = robot.Arm.getCurrentPosition();
        int ArmHoldPosition = robot.Arm.getCurrentPosition();
        int LinearSlideHold = robot.LinearSlide.getCurrentPosition();
        double ClawTargetPosition = 0.40; // 1 is equal to 180 degrees of rotation
        double r, g, b;


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double time = getRuntime();
            double tdist;
            // Drivetrain
            robot.driveWithControllers(-Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x,
                    -1 * Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y,
                    Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x, 1 - 0.5 * gamepad1.left_trigger);


            // Intake
            if (gamepad2.right_trigger > 0.05) robot.Intake.setPower(1 * gamepad2.right_trigger);
            else if (gamepad2.left_trigger > 0.05) robot.Intake.setPower(-1 * gamepad2.left_trigger);
            else robot.Intake.setPower(0);


            // Arm
            if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                robot.Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Arm.setPower(-0.5 * gamepad2.right_stick_y);
                ArmHoldPosition = robot.Arm.getCurrentPosition();
            } else {
                robot.Arm.setPower(1);
                robot.Arm.setTargetPosition(ArmHoldPosition + StartingArmPosition);
                robot.Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            // Linear Slide
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                robot.LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.LinearSlide.setPower(gamepad2.left_stick_y);
                LinearSlideHold = robot.LinearSlide.getCurrentPosition();
            } else {
                robot.LinearSlide.setPower(1);
                robot.LinearSlide.setTargetPosition(LinearSlideHold);
                robot.LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //Plane
            //servo runs for 5 cycles
            if(gamepad2.y) {
                tdist= time;
                if(tdist+5>time){
                    robot.Plane.setPower(1);
                }else{robot.Plane.setPower(0);}
            }



            // Claw
            if(gamepad2.x) robot.Claw.setPower(1);
            else if(gamepad2.a) robot.Claw.setPower(-1);
            else if(gamepad2.b) robot.Claw.setPower(0);

            r =robot.cSensor.red();
            g =robot.cSensor.green();
            b =robot.cSensor.blue();

            robot.colorCheck(r, g, b);


            telemetry.addData("Claw Pos", robot.Claw.getPower());
            telemetry.addData("Intake", robot.Intake.getPower());
            telemetry.addData("LinearSlides", robot.LinearSlide.getCurrentPosition());
            telemetry.addData("Arm", robot.Arm.getCurrentPosition());
            telemetry.addData("LF", robot.LF.getPower());
            telemetry.addData("RF", robot.RF.getPower());
            telemetry.addData("LB", robot.LB.getPower());
            telemetry.addData("RB", robot.RB.getPower());
            telemetry.update();
        }
    }
}


class RobotHardware {

    public final HardwareMap map;
    public final Telemetry telemetry;


    public final BNO055IMU imu;


    public final DcMotor RF, RB, LF, LB;


    public final DcMotor Intake, LinearSlide, Arm;


    public final CRServo Claw, Plane;
    public final ColorSensor cSensor;

    public final DistanceSensor dSensor0, dSensor1;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // "deviceName:" is what appears in the configure section on control hub
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        LinearSlide = hardwareMap.get(DcMotor.class, "LinearSlide");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        dSensor0= hardwareMap.get(DistanceSensor.class, "DS0");
        dSensor1= hardwareMap.get(DistanceSensor.class, "DS1");
        Claw = hardwareMap.get(CRServo.class, "Claw");
        Plane = hardwareMap.get(CRServo.class, "Plane");


        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        LinearSlide.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);

        Claw.setDirection(CRServo.Direction.FORWARD);


        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setTargetPosition(Arm.getCurrentPosition());
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLedDriver");


        //declare sensors
        cSensor = hardwareMap.get(ColorSensor.class, "cSensor");


        // Information about what the stuff above does: https://ftc-tricks.com/dc-motors/

        telemetry.addData("Status", "Robot Hardware Initialized");
        telemetry.update();
        this.map = hardwareMap;


    } // initializes everything


    public void MoveDirection(double direction, double turn, double throttle, long time) {
        // tile = ~23 inches

        double forward = Math.cos(Math.PI * (direction / 180));
        double strafe = Math.sin(Math.PI * (direction / 180));
        double max_power = Math.max(1, Math.max(Math.max(
                -forward - strafe + turn, // LF
                -forward + strafe + turn // LB
        ), Math.max(
                -forward + strafe - turn, // RF
                -forward - strafe - turn // RB
        )));
        strafe /= max_power;
        forward /= max_power;
        turn /= max_power;
        LF.setPower(throttle * (-forward - strafe + turn));
        LB.setPower(throttle * (-forward + strafe + turn));
        RF.setPower(throttle * (-forward + strafe - turn));
        RB.setPower(throttle * (-forward - strafe - turn));

        try {
            Thread.sleep(time * 1000);
        } catch (InterruptedException e) {
            // Wait the set amount of time before stopping motors
        }

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }


    public void driveWithControllers(double strafe, double forward, double turn, double throttle) {
        double max_power = Math.max(1, Math.max(Math.max(
                Math.abs(-forward - strafe + turn), // LF
                Math.abs(-forward + strafe + turn) // LB
        ), Math.max(
                Math.abs(-forward + strafe - turn), // RF
                Math.abs(-forward - strafe - turn) // RB
        )));
        strafe /= max_power;
        forward /= max_power;
        turn /= max_power;
        LF.setPower(throttle * (-forward - strafe + turn));
        LB.setPower(throttle * (-forward + strafe + turn));
        RF.setPower(throttle * (-forward + strafe - turn));
        RB.setPower(throttle * (-forward - strafe - turn));


    }
    public void colorCheck (double red, double green, double blue){
        // check for White, Black, yellow, Green, Purple
        if (red > 230 && green > 230 && blue > 230) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        } else if (red < 60 && green < 60 && blue < 60){
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        } else if  (red > blue && green > blue) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        }  else if (green > red && green > blue) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (blue > green && red > green) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
            // Non pixel colors output strobe red for error
        }
    }
    public void fakeColorCheck (boolean up, boolean down, boolean right, boolean left){
        if (up){pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;}
        if (down){pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;}
        if (right){pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;}
        if (left){pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;}
    }
    public int DSCheck (double dista, double distb){
        if (dista < 24){return 1;}
        if (distb < 24){return 3;}
        else{return 2;}
    }
}


