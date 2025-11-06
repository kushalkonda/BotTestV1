//package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous
//public class AutonV1 extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    public int pathState;
//    private final Pose startPose = new Pose(55.425, 136, Math.toRadians(270));
//    private final Pose shootPose = new Pose(55.425, 95, Math.toRadians(135));
//    private final Pose intake1Pose = new Pose ( 25, 84, Math.toRadians(180));
//    private final Pose intake2Pose = new Pose( 25, 59, Math.toRadians(180));
//    private final Pose intake3Pose = new Pose(25, 35, Math.toRadians(180));
//
//    //path variables
//    private Path startToScore;
//    private PathChain intake1AndScore, intake2AndScore, intake3AndScore;
//
//    //Subsystems
//    DcMotor intake, shooter, turret;
//    CRServo servo1, servo2;
//    Servo kicker;
//    double pos1 = .8; //servo kicker pos
//    double pos2 = .88;
//    ColorSensor colorSensor;
//    int threshold = 200;//color sensor threshold
//    int ballCount = 0;//count for colorSensor
//    boolean ballPresent = false;
//    boolean ballWasPresent = false;
//    ElapsedTime shootingTimer = new ElapsedTime();
//    boolean timerStart = false;
//    boolean shooting; //used for shooting updates and pedropathing
//
//
//    @Override //main method
//    public void loop(){
//        follower.update();
//        autonomousPathUpdate();;
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//
//    }
//
//
//    public void buildPaths(){
//        //Start to Score path. BezierLine path 1
//        startToScore = new Path(new BezierLine(startPose, shootPose));
//
//        //path 2
//        intake1AndScore = follower.pathBuilder()
//                .addPath(new BezierCurve (shootPose, intake1Pose, /*control point */ new Pose (25, 84)))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
//                .addPath(new BezierLine (intake1Pose, shootPose))
//                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shootPose.getHeading())
//                .build();
//
//        //path 3
//        intake2AndScore = follower.pathBuilder()
//                .addPath(new BezierCurve (shootPose, intake2Pose, /*control point */ new Pose (85.6, 58.9)))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), intake2Pose.getHeading())
//                .addPath(new BezierLine (intake2Pose, shootPose))
//                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose.getHeading())
//                .build();
//
//        //path 4
//        intake3AndScore = follower.pathBuilder()
//                .addPath(new BezierCurve (shootPose, intake3Pose, /*control point */ new Pose (82.82, 32.7)))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), intake3Pose.getHeading())
//                .addPath(new BezierLine (intake3Pose, shootPose))
//                .setLinearHeadingInterpolation(intake3Pose.getHeading(), shootPose.getHeading())
//                .build();
//
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState){
//            case 0: follower.followPath(startToScore);
//                shoot(.7);
//                setPathState(1);
//                break;
//            case 1:
//                if (!follower.isBusy() && !shooting){
//                    follower.followPath(intake1AndScore);
//                    shoot(.7);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if (!follower.isBusy() && !shooting) {
//                    follower.followPath(intake2AndScore);
//                    shoot(.7);
//                    setPathState(3);
//                }
//                break;
//            case 3 :
//                if (!follower.isBusy() && !shooting){
//                    follower.followPath(intake3AndScore);
//                    shoot(.7);
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//    public void setPathState(int pState){
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//}