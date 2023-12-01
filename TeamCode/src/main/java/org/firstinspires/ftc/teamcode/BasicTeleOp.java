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
        int StartingArmPosition = robot.Arm0.getCurrentPosition();
        int ArmHoldPosition = robot.Arm0.getCurrentPosition();
        int StartingArmPosition1 = robot.Arm1.getCurrentPosition();
        int ArmHoldPosition1 = robot.Arm1.getCurrentPosition();
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
                robot.Arm0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Arm0.setPower(-0.5 * gamepad2.right_stick_y);
                robot.Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Arm1.setPower(-0.5 * gamepad2.right_stick_y);
                ArmHoldPosition = robot.Arm0.getCurrentPosition();
                ArmHoldPosition1 = robot.Arm1.getCurrentPosition();
            } else {
                robot.Arm0.setPower(1);
                robot.Arm0.setTargetPosition(ArmHoldPosition + StartingArmPosition);
                robot.Arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.Arm1.setPower(1);
                robot.Arm1.setTargetPosition(ArmHoldPosition1 + StartingArmPosition1);
                robot.Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            telemetry.addData("Arm", robot.Arm0.getCurrentPosition());
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
    static final double EncoderTickToTile =  1/(( 23.4 * 25.4 * 4096 ) / 109.955742876);


    public final BNO055IMU imu;


    public final DcMotor RF, RB, LF, LB;


    public final DcMotor Intake, LinearSlide, Arm0, Arm1;


    public final CRServo Claw, Plane;
    public final ColorSensor cSensor;

    public final DistanceSensor dSensor0, dSensor1;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    //c1 and C2 are approx
    static final double C1 = 2.72824076488, C2 = 1.82212373908, mmToTile =
            0.00168248199744, EncoderTickToMM = 0.026844663788,
    //Encoderticktomm is constant and fixed- no calibration
    //mmtotile is constant + fixed- no calibration
    //C2 is updated
    //C1 is not used agian
            encoderBad = (1 * 4096) / (1 * 4096), // (tiles supposed to go * ticks supposed to go) / (distance told to go * encoder ticks actually gone)
    //amount of error in encoder for calibration
    //needs to be updated to our robot
    encoderTicksPer360 = 3.11784486041 * 22708.224; // encoder wheel distance for 360 degrees rotation in tile lengths * tile to encoder
    //1st term is maybe updated
    //2nd term is updated to our encoders

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
        Arm0 = hardwareMap.get(DcMotor.class, "Arm");
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
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
        Arm0.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm1.setDirection(DcMotorSimple.Direction.REVERSE);

        Claw.setDirection(CRServo.Direction.FORWARD);


        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm0.setTargetPosition(Arm0.getCurrentPosition());
        Arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm1.setTargetPosition(Arm1.getCurrentPosition());
        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Arm0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLedDriver");


        //declare sensors
        cSensor = hardwareMap.get(ColorSensor.class, "cSensor");
        cSensor.enableLed(false);



        // Information about what the stuff above does: https://ftc-tricks.com/dc-motors/

        telemetry.addData("Status", "Robot Hardware Initialized");
        telemetry.update();
        this.map = hardwareMap;


    } // initializes everything
    public double[] updateCoords(double[] CurrentCoords) {

        double e1 = (-(LF.getCurrentPosition() * encoderBad) - CurrentCoords[3])
                * EncoderTickToMM;
        double e2 = ((RF.getCurrentPosition() * encoderBad) - CurrentCoords[4])
                * EncoderTickToMM;
        double e3 = ((LB.getCurrentPosition() * encoderBad) - CurrentCoords[5])
                * EncoderTickToMM;
        double ChangeInRotation = 0; //((e1 - e2) / (2 * C1));
        return new double[]{((e3 + ChangeInRotation * C2) * Math.cos((Math.PI *
                CurrentCoords[2]) / 180) + ((e1 + e2) / 2) * Math.sin((Math.PI *
                CurrentCoords[2]) / 180) + (CurrentCoords[0] / mmToTile)) * mmToTile, // X
                (-1 * (e3 + ChangeInRotation * C2) * Math.sin((Math.PI *
                        CurrentCoords[2]) / 180) + ((e1 + e2) / 2) * Math.cos((Math.PI *
                        CurrentCoords[2]) / 180) + (CurrentCoords[1] / mmToTile)) * mmToTile, // Y
                (ChangeInRotation + CurrentCoords[2]), // Rotation Z
                -LF.getCurrentPosition() * encoderBad, RF.getCurrentPosition() *
                encoderBad, LB.getCurrentPosition() * encoderBad}; // Last Encoder Positions
    }

    public double[] driveToTile(double[] CurrentCoords, double[] TargetCoords, double Speed) {
        //inputs are arrays, [x,y,turn], [x,y,turn], speed(0-1)
        CurrentCoords = updateCoords(CurrentCoords);

        double Distance = Math.sqrt(Math.pow(TargetCoords[0] - CurrentCoords[0],
                2) + Math.pow(TargetCoords[1] - CurrentCoords[1], 2));
        double maxPower = 0, maxSpeed = 0;
        //macro section of PID loop
        while (Distance > 0.05) { // ||
            //Math.abs(angleDifference(CurrentCoords[2], TargetCoords[2])) > 5

            CurrentCoords = updateCoords(CurrentCoords);
            Distance = Math.sqrt(Math.pow(TargetCoords[0] - CurrentCoords[0], 2)

                    + Math.pow(TargetCoords[1] - CurrentCoords[1], 2));

// Tell motors what to do
            maxPower = Math.min(1, Math.max(Math.max(Math.abs(TargetCoords[0] -
                    CurrentCoords[0]), Math.abs(TargetCoords[1] - CurrentCoords[1])), 0.0001));
            maxSpeed = Math.min(1, Math.max((Distance / 0.75), 0.2)); //
            //Distance in Tiles when motors start slowing down, minimum speed

            driveWithControllers(-1 * ((TargetCoords[0] - CurrentCoords[0]) /

                            maxPower) * maxSpeed,

                    ((TargetCoords[1] - CurrentCoords[1]) / maxPower) *

                            maxSpeed,

                    0.5 * TargetCoords[2], // 0.5 *
                    //(angleDifference(TargetCoords[2], CurrentCoords[2]) / 180);

                    Speed);
            //update telem to help w/ tuning
            telemetry.addData("Distance", Distance);
            telemetry.addData("Angle Diff:", angleDifference(CurrentCoords[2], TargetCoords[2]));

            telemetry.addData("X:", CurrentCoords[0]);
            telemetry.addData("Y:", CurrentCoords[1]);
            telemetry.addData("Z:", CurrentCoords[2]);
            telemetry.addData("X Power", -1 * ((TargetCoords[0] - CurrentCoords[0]) / maxPower) * maxSpeed);

            telemetry.addData("Y Power", ((TargetCoords[1] - CurrentCoords[1]) / maxPower) * maxSpeed);

            telemetry.addData("Z Power",0.5 * (angleDifference(TargetCoords[2], CurrentCoords[2]) / 180) * maxSpeed);
            telemetry.update();
        }
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
// Prevents updating encoders while robot is still moving
        methodSleep(300);
        return updateCoords(CurrentCoords);
    }
//turning
    public double[] turnDegrees(double[] CurrentCoords, double degrees) {
        double leftStart = LF.getCurrentPosition() * encoderBad;
        double rightStart = RF.getCurrentPosition() * encoderBad;
        double targetRotation = Math.abs(encoderTicksPer360 * (degrees / 360));
        while (Math.abs(LF.getCurrentPosition() * encoderBad - leftStart) <
                targetRotation && Math.abs(RF.getCurrentPosition() * encoderBad - rightStart) <
                targetRotation) {

            driveWithControllers(0, 0, Math.signum(degrees) * 0.5, 0.5);
        }
        driveWithControllers(0, 0, 0, 0);
        methodSleep(300);

        CurrentCoords[2] = angleDifference(0, degrees + CurrentCoords[2]);
        return CurrentCoords;
    }

    public double angleDifference(double CurrentAngle, double TargetAngle) {
        double result1 = Math.floorMod(Math.round((TargetAngle - CurrentAngle) *
                100), 360 * 100) * 0.01;
        double result2 = Math.floorMod(Math.round((TargetAngle - CurrentAngle) *
                100), -360 * 100) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }

    public void methodSleep(long time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
// Wait the set amount of time before stopping motors
        }
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

        if (red > 210 && green > 210 && blue > 210) {
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
        blinkinLedDriver.setPattern(pattern);


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
}


