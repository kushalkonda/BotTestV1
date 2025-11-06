package org.firstinspires.ftc.teamcode.exclude;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class Camera extends LinearOpMode {
    AprilTagProcessor MyApriltagProcessor;
    VisionPortal myVisionPortal;

// Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            AprilTagDetection myAprilTagDetection;
            List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();
            if (detections.size() > 0) {
                // Take the first detected tag for telemetry
                AprilTagDetection tag = detections.get(0);
                telemetry.addLine("------");
                telemetry.addData("Tag ID", tag.id);
                //telemetry.addData("Range", tag.ftcPose.range);
                telemetry.addData("Range (in)", "%.2f", tag.ftcPose.range);


            } else {
                telemetry.addLine("No tags detected");
            }

            telemetry.update();
        }
    }
}


