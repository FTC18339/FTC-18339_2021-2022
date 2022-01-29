package org.firstinspires.ftc.teamcode;

import java.util.Set;

public class FTC_18339_AutonomousProtocol002 extends Main {

    public Command[] commands;
    boolean runningAuto = false;
    int commandsIndex = 0;

    public float mmPerInch = Algorithms.mmPerInch;

    //Carosel algorithm distances
    public double firstMoveC = 21 * mmPerInch;
    public double secondMoveC = 14 * mmPerInch;
    public double thirdMoveC = 48 * mmPerInch;
    public double fourthMoveC = 24 * mmPerInch;


    @Override
    public void runOpMode() {
        initMaths();
        initHardware();
        initManualModes();

        initAutonomousModes();

        ChildCommandInitialization();

        waitForStart();

        gripper1.setPosition(1);

        if(opModeIsActive()) {
            while(opModeIsActive()) {
                if(!runningAuto) {
                    runningAuto = true;
                    if(commandsIndex < commands.length) {
                        RunAutoCommand(commands[commandsIndex]);
                    } else
                        break;

                    StopMotors();
                }
                idle();
            }
        }
    }

    //Experimentally tested value
    double rotationAngleOfOneRevolution = Math.toRadians(55.1);
    public void RunAutoCommand(Command command) {
        String name = command.name;
        double data = command.data;
        long time = System.currentTimeMillis() + (long)(command.time * 1000);

        Vector3 IKTarget = math.startPosition;

        float ticksForMotors = MAX_NUM_TICKS_MOVEMENT;
        double revs = data / Algorithms002.wheelCircumferenceMm;

        if(command.positional) {
            switch (name) {
                case "ROTATE":
                    //Find the revolution the wheels must take for a certain angle, use the desired angle and divide by the roation that
                    //one wheel revolution provides.
                    revs = data / rotationAngleOfOneRevolution;
                    break;
                case "ONEREVROT":
                    revs = 1;
                    break;
                case "IKSTARTPOSITION":
                    IKTarget = math.startPosition;
                    break;
                case "IKTOPDROP":
                    IKTarget = math.topPosition;
                    break;
            }
        }

        int ticks = (int)(revs * ticksForMotors);

        if(name == "MOVE") {
            SetTicksAndMotorsForMovement(ticks, false);
        } else if (name == "ROTATE" || name == "ONEREVROT") {
            SetTicksAndMotorsForMovement(ticks, true);
        } else if(name == "IKSTARTPOSITION") {
            SetIKArmForTarget(0, 1, 0);
        } else if(name == "IKTOPDROP") {
            SetIKArmForTarget(0, 3/5, 1);
        } else if(name == "GRIPPERDROP") {
            GripperDrop();
        } else if (name == "REDSPINNER") {
            Spinner(true);
        } else if(name == "BLUESPINNER") {
            Spinner(false);
        }

        gripper1.setPosition(1);
        commandsIndex++;
        runningAuto = false;
    }

    void SetTicksAndMotorsForMovement(int ticks, boolean rot) {
        int rotMultiplier = 1;
        if(rot) rotMultiplier = -1;

        left_back.setTargetPosition(rotMultiplier * ticks);
        left_front.setTargetPosition(ticks);
        right_back.setTargetPosition(rotMultiplier * ticks);
        right_front.setTargetPosition(ticks);
        RunToPositionAutonomousMovement();

        double ticksSpeed = MAX_NUM_TICKS_MOVEMENT * 0.05 * MOVEMENT_RPM;

        left_back.setVelocity(rotMultiplier * ticksSpeed);
        left_front.setVelocity(ticksSpeed);
        right_back.setVelocity(ticksSpeed);
        right_front.setVelocity(rotMultiplier * ticksSpeed);

        while(left_back.isBusy() && opModeIsActive()) {

        }
    }

    void SetIKArmForTarget(double x, double y, double phi) {
        //double[] qs = math.IKArm(x, y, phi);

        base_arm_joint.setPosition(1 - x);
        second_arm_joint.setPosition(y);
        hand.setPosition(1 - (phi / (3/2)));
        gripper1.setPosition(0.8f);

        long time = System.currentTimeMillis();
        while((System.currentTimeMillis() <= time + 500) && opModeIsActive()) {

        }
    }

    void GripperDrop() {
        gripper1.setPosition(0.4f);

        long time = System.currentTimeMillis();
        while((System.currentTimeMillis() <= time + 300) && opModeIsActive()) {

        }
    }

    void Spinner(boolean red) {
        int dir = 1;

        if(red) {
            dir = -1;
        }

        spinner.setPower(dir);

        long time = System.currentTimeMillis();
        while((System.currentTimeMillis() <= time + 8000) && opModeIsActive()) {

        }
    }

    public void ChildCommandInitialization() {}
}
