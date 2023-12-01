package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.DistanceSensor;
//import org.firstinspires.ftc.teamcode.BasicTeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
@Autonomous(group = "drive")

public class autoroadruntest extends LinearOpMode{




    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .strafeRight(3)
                    .build();
            drive.followTrajectorySequence(trajSeq);

        }
    }
    public int DSCheck (double dista, double distb){
        if (dista < 24){return 1;}
        if (distb < 24){return 3;}
        else{return 2;}
    }

}
