package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Autonomous operation", group = "Competition")
public class AutonomousPhase extends OpMode {

    private HardwareInitialiser h;
    private int quadrant;
    private String alliance;
    private Locator l;

    @Override
    public void init() {
        l = new Locator(hardwareMap);

        /*
         * Get yaw and calculate quadrant.
         * Assumes the robot is pointing directly away from the centre of the game area.
         */
        waitForFoundTarget(l);
        float yaw = l.getRotation().thirdAngle;
        quadrant = (int) Math.ceil(yaw / 90);

        /* Blue alliance is in quadrants 1 and 2, red alliance is in quadrants 0 and -1 */
        alliance = (quadrant >= 1) ? "Blue" : "Red";

        telemetry.addData("Quadrant", "%d", quadrant);
        telemetry.addData("Alliance", "%s", alliance);
        telemetry.addData("Status", "Initialised");
        telemetry.update();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        h = new HardwareInitialiser(hardwareMap);
        h.runtime.reset();
    }

    @Override
    public void loop() {
        waitForFoundTarget(l);
        telemetry.addData("yaw", "%f", l.getRotation().thirdAngle);
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    private void waitForFoundTarget(Locator locator){
        do {
            locator.process();
        } while(locator.getVisibleTarget().equals("none")); // process the data until a target is found
    }
}
