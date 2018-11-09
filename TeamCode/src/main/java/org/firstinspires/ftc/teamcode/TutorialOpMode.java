package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.sun.tools.doclint.Entity.not;

@TeleOp
public class TutorialOpMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorTest;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColourRange;
    private Servo servoTest;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorTest = hardwareMap.get(DcMotor.class, "motor 0 HD hex");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch sensor");
        sensorColourRange = hardwareMap.get(DistanceSensor.class, "rev colour");
        servoTest = hardwareMap.get(Servo.class, "srs0");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();

        double tgtPower;
        while(opModeIsActive()) {
            tgtPower = -this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);

            if(gamepad1.y) {
                servoTest.setPosition(0);
            } else if(gamepad1.x || gamepad1.b) {
                servoTest.setPosition(0.5);
            } else if(gamepad1.a) {
                servoTest.setPosition(1);
            }

            if(!digitalTouch.getState()) {
                telemetry.addData("Button", "On");
            } else {
                telemetry.addData("Button", "Off");
            }

            telemetry.addData("Target power", tgtPower);
            telemetry.addData("Motor power", motorTest.getPower());
            telemetry.addData("Distance (cm)", sensorColourRange.getDistance(DistanceUnit.CM));
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
