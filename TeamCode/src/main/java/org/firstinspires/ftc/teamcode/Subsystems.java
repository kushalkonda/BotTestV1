package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Subsystems {
    private DcMotor intake, turret;
    private DcMotorEx shooter;
    private CRServo servo1,servo2;
    private Servo kicker;
    private VoltageSensor volts;
    private Gamepad gamepad1;

    double kickerDown = .175;//kicker positions
    double kickerUp = .45;

    int ballCount = -1;
    int shootingState = -1;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timerShooter = new ElapsedTime();
    private double lastVelocity = 0;
    private double lastTime =0;
    public static double threshold = 4000;
    public static double increment =100;
    private int veloDrops = 0;
    public static double velocityGoal = 2500;
    double slope;


    boolean lastBState = false;
    public Subsystems(HardwareMap hardwareMap,Gamepad gamepad1) {
        intake = hardwareMap.get(DcMotor.class,"int");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//        turret = hardwareMap.dcMotor.get("tur");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo1 = hardwareMap.crservo.get("rs");
        servo2 = hardwareMap.crservo.get("ls");
        kicker = hardwareMap.servo.get("s1");
        volts= hardwareMap.get(VoltageSensor.class,"Control Hub");
        this.gamepad1 = gamepad1;
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
    }

    public void rollers(double intakeSpeed, double servoSpeed){
        intake.setPower(-intakeSpeed);
        servo1.setPower(-servoSpeed);
        servo2.setPower(servoSpeed);
    }
    public void kicker (boolean up){
        if (up){
            kicker.setPosition(kickerUp);
        }else{
            kicker.setPosition(kickerDown);
        }
    }
    public void shoot(){
        if (shootingState == -1){
            shootingState = 0;
            ballCount = 0;
        }
    }
    public double getVelocity(){
        return shooter.getVelocity();
    }

    private boolean velocityDropped(){
        if(timerShooter.milliseconds()-lastTime>increment){
            lastTime=timerShooter.milliseconds();
            slope = (shooter.getVelocity()*(60/28.0)-lastVelocity)/(increment);
        }
        lastVelocity = shooter.getVelocity()*(60.0/28);
        if(slope<-threshold){
            veloDrops++;
            return true;
        }
        return false;

    }

    public double  getSlope(){
        return (shooter.getVelocity()*(60/28.0)/velocityGoal);
    }

    public int getVeloDrops(){
        return veloDrops;
    }

    public void shooterUpdate(){
        if (gamepad1.bWasPressed()){
            ballCount++;
        }
        lastBState = gamepad1.b;
        switch(shootingState){
            case 0: shooter.setVelocity(velocityGoal*(28.0/60));
                shootingState = 1;
                break;
            case 1: //check for velocity reached
                if (shooter.getVelocity()*(60.0/28)>2400) {
                    rollers(1,1);
                    shootingState = 2;
                }
                break;
            case 2: //ball detection
                if(shooter.getVelocity()*(60/28.0)/velocityGoal<(2000.0/2500)){
                    ballCount++;
                }
                if (ballCount == 1){
                    rollers(0,-.2);
                    timer.reset();
                    shootingState = 3;
                }
                break;
            case 3:
                if (shooter.getVelocity()*(60.0/28)>velocityGoal){
                    rollers(1,1);
                    shootingState= 4;
                }

                break;

            case 4:
                if(shooter.getVelocity()*(60/28.0)/velocityGoal<(2000.0/2500)){
                    ballCount++;
                }
                if (ballCount==2){
                    rollers(0,-.2);
                    timer.reset();
                    shootingState = 5;
                }
                break;
            case 5:
                if (shooter.getVelocity()*(60.0/28)>velocityGoal){
                    rollers(1,1);
                    shootingState= 6;
                }

                break;
            case 6:
                if (shooter.getVelocity()*(60.0/28)>velocityGoal){
                    kicker(true);

                }
                if(shooter.getVelocity()*(60/28.0)/velocityGoal<(2000.0/2500)){
                    ballCount=0;
                    shootingState=7;
                    kicker(false);
                    rollers(1,0);
                    shootingState = -1;
                }
                break;
//            case 7:
//                if (ballCount==0){
//                    kicker(false);
//                    rollers(1,0);
//                    shootingState=-1;
//                }
//                break;
        }
    }
}

