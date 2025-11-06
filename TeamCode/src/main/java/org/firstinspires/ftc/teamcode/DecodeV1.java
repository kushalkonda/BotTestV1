package org.firstinspires.ftc.teamcode;

import com.bylazar.graph.GraphManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.graph.PanelsGraph;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class DecodeV1 extends LinearOpMode {
    private GraphManager graphManager;
    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {
         Subsystems subsystem = new Subsystems(hardwareMap,gamepad1);
         Drive drive  = new Drive(hardwareMap);
         telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
         graphManager = PanelsGraph.INSTANCE.getManager();
         subsystem.kicker(false);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //dt
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            drive.move(y, x, rx);
            //intake
            if (subsystem.shootingState == -1){
                if (gamepad1.right_trigger > .1){
                    subsystem.rollers(gamepad1.right_trigger, 0);
                }else if (gamepad1.left_trigger > .1){
                    subsystem.rollers((gamepad1.left_trigger), 0);
                }else {
                    subsystem.rollers(0,0);
                }
            }
            if (gamepad1.xWasPressed()){
                subsystem.kicker(true);
            }else{
                subsystem.kicker(false);
            }
            //Shooter
            if (gamepad1.aWasPressed()){
                subsystem.shoot();
            }
            subsystem.shooterUpdate();
//            telemetryM.addData("velocity", subsystem.getVelocity()*(60.0/28));
//            telemetryM.addData("drops", subsystem.getVeloDrops());
//            telemetryM.addData("slope", subsystem.getSlope());
//            graphManager.addData("velo", subsystem.getVelocity()*(60.0/28));
//            telemetryM.update();
//            graphManager.update();
        }
    }
}


