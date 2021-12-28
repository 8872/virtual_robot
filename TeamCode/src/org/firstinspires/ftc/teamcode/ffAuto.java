package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//trajectories image https://trello.com/1/cards/61b153528c22031568d19dd9/attachments/61b153528c22031568d19de5/previews/61b153538c22031568d19e22/download/image.png
@Autonomous(name = "Freight Frenzy Auto")
public class ffAuto extends LinearOpMode {
    DcMotor m1, m2, m3, m4;
    static boolean red = false;
    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajectory0R = drive.trajectorySequenceBuilder(new Pose2d())
                .back(54)
                .lineToLinearHeading(new Pose2d(-33, -60, Math.toRadians(90)))
                .build();

        TrajectorySequence trajectory0B = drive.trajectorySequenceBuilder(new Pose2d())
                .back(54)
                .lineToLinearHeading(new Pose2d(-33, 60, Math.toRadians(-90)))
                .build();
        if(red){
            drive.followTrajectorySequence(trajectory0R);
        } else{
            drive.followTrajectorySequence(trajectory0B);
        }
        //red
        Trajectory trajectory1R = drive.trajectoryBuilder(trajectory0R.end())
                .lineToLinearHeading(new Pose2d(-50, -60, Math.toRadians(0)))
                .build();
        Trajectory trajectory2R = drive.trajectoryBuilder(trajectory1R.end())
                .lineToLinearHeading(new Pose2d(-12, -46, Math.toRadians(-90)))
                .build();
        Trajectory trajectory3R = drive.trajectoryBuilder(trajectory2R.end())
                .splineToLinearHeading(new Pose2d(54, -54, Math.toRadians(0)), Math.toRadians(0))
                .build();
        //blue
        Trajectory trajectory1B = drive.trajectoryBuilder(trajectory0B.end())
                .lineToLinearHeading(new Pose2d(-50, 60, Math.toRadians(-90)))
                .build();
        Trajectory trajectory2B = drive.trajectoryBuilder(trajectory1B.end())
                .lineToLinearHeading(new Pose2d(-12, 46, Math.toRadians(90)))
                .build();
        Trajectory trajectory3B = drive.trajectoryBuilder(trajectory2B.end())
                .splineToLinearHeading(new Pose2d(54, 54, Math.toRadians(0)), Math.toRadians(0))
                .build();




        waitForStart();

        if (isStopRequested()) return;

        if(red) {
            drive.followTrajectory(trajectory1R);
            //turn on carousel
            drive.followTrajectory(trajectory2R);
            //drop off payload (pulley, arm)
            drive.followTrajectory(trajectory3R);
        } else{
            drive.followTrajectory(trajectory1B);
            //turn on carousel
            drive.followTrajectory(trajectory2B);
            //drop off payload (pulley, arm)
            drive.followTrajectory(trajectory3B);
        }



        while (!isStopRequested() && opModeIsActive()) ;
    }


}