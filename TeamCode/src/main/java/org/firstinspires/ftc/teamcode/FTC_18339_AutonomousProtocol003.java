package org.firstinspires.ftc.teamcode;

public class FTC_18339_AutonomousProtocol003 extends Main002 {

    public Command[] commands;
    boolean runningAuto = false;
    int commandsIndex = 0;

    public float mmPerInch = Algorithms.mmPerInch;

    //Carosel algorithm distances
    public double firstMoveC = 19 * mmPerInch;
    public double secondMoveC = 14 * mmPerInch;
    public double thirdMoveC = 42 * mmPerInch;
    public double fourthMoveC = 24 * mmPerInch;
    public double rotationC = 50;

    //Warehouse algorithm distances
    public double thirdMoveW = 12 * mmPerInch;
    public double fourthMoveW = 45 * mmPerInch;


    @Override
    public void runOpMode() {
        initMaths();
        initHardware();
        initManualModes();

        initAutonomousModes();

        ChildCommandInitialization();

        waitForStart();

        gripper1.setPosition(0.8f);

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
        long time = System.currentTimeMillis() + (long) (command.time * 1000);

        Vector3 IKTarget = math.startPosition;

        float ticksForMotors = MAX_NUM_TICKS_MOVEMENT;
        double revs = data / Algorithms002.wheelCircumferenceMm;

        if (command.positional) {
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

        int ticks = (int) (revs * ticksForMotors);

        if (name == "MOVE") {
            SetTicksAndMotorsForMovement(ticks, false);
        } else if (name == "ROTATE" || name == "ONEREVROT") {
            SetTicksAndMotorsForMovement(ticks, true);
        } else if (name == "IKSTARTPOSITION") {
            //SetIKArmForTarget(0, 1, 0);
        } else if (name == "IKTOPDROP") {
            SetIKArmForTop();
        } else if(name == "IKFORSTART") {
            SetIKArmForStart();
        } else if(name == "GRIPPERDROP") {
            GripperDrop();
        } else if (name == "REDSPINNER") {
            Spinner(true);
        } else if(name == "BLUESPINNER") {
            Spinner(false);
        }

        initAutonomousModes();
        spinner.setPower(0);
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

    void SetIKArmForTop() {
        //double[] qs = math.IKArm(x, y, phi);

        double craneHeight = 0f;
        double baseHeight = 0;
        double phi = 0.5;

        double q1 = (baseHeight + 1) / 2;
        double q2 = (craneHeight + 1) / 2;
        double q3 = (phi + 1) / 2;

        base_arm_joint.setPosition(q1);
        second_arm_joint.setPosition((q2 / (3/2)));
        hand.setPosition(q3 / (3/2));
        gripper1.setPosition(0.8f);

        long time = System.currentTimeMillis();
        while((System.currentTimeMillis() <= time + 2000) && opModeIsActive()) {

        }
    }

    void SetIKArmForStart() {
        double craneHeight = -1f;
        double baseHeight = 0;
        double phi = 0.5;

        double q1 = (baseHeight + 1) / 2;
        double q2 = (craneHeight + 1) / 2;
        double q3 = (phi + 1) / 2;

        base_arm_joint.setPosition(1 - q1);
        second_arm_joint.setPosition(1 - (q2 / (3/2)));
        hand.setPosition(q3 / (3/2));
        gripper1.setPosition(0.4f);

        long time = System.currentTimeMillis();
        while((System.currentTimeMillis() <= time + 2000) && opModeIsActive()) {

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

        spinner.setPower(dir * 0.5);

        long time = System.currentTimeMillis();
        while((System.currentTimeMillis() <= time + 8000) && opModeIsActive()) {

        }
    }

    public void ChildCommandInitialization() {}
}
