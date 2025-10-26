package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DecodeV1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake = hardwareMap.dcMotor.get("int");
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        DcMotor turret = hardwareMap.dcMotor.get("tur");

        //dt motors
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");


        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        VoltageSensor volts= hardwareMap.get(VoltageSensor.class,"Control Hub");
        CRServo servo1,servo2;
        Servo kicker,stopper;
        servo1 = hardwareMap.crservo.get("rs");
        servo2 = hardwareMap.crservo.get("ls");
        kicker = hardwareMap.servo.get("s1");
        //stopper = hardwareMap.servo.get("s2");
        double servoPosition = 0.5;
        double pow = .01;
        int lastPos=0;






        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            intake.setPower(-gamepad1.right_trigger);
            //shooter.setPower(.97*(12/volts.getVoltage()));
            servo1.setPower(-gamepad1.left_trigger);
            servo2.setPower(gamepad1.left_trigger);
            double pos1 = .8;
            double pos2 = .88;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeft.setPower((y + x + rx) / denominator);
            backLeft.setPower((y - x + rx) / denominator);
            frontRight.setPower((y - x - rx) / denominator);
            backRight.setPower((y + x - rx) / denominator);




            if(gamepad1.a){
                kicker.setPosition(pos2);

            }
            else if(gamepad1.b) {
                kicker.setPosition(pos1);
            }
            if (gamepad1.dpadUpWasPressed())
            {
                shooter.setPower(.7*(12/volts.getVoltage()));
            }else if (gamepad1.dpadDownWasPressed())
            {
                shooter.setPower(0);
            }

            telemetry.addData("Servo power", servo1.getPower());
            telemetry.addData("Power", shooter.getPower());
            telemetry.update();

        }
    }
}


