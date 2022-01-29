package org.firstinspires.ftc.teamcode;

public class FTC_18339_AutonomousProtocol001 extends Main {

    public Command[] commands;
    boolean runningAuto = false;
    int commandsIndex = 0;

    public float mmPerInch = Algorithms.mmPerInch;

    @Override
    public void runOpMode() {
        initMaths();
        initHardware();
        initManualModes();

        initAutonomousModes();
        SetAutonomousDirection();

        ChildCommandInitialization();

        waitForStart();

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
    double rotationAngleOfOneRevolution = Math.toRadians(30);
    public void RunAutoCommand(Command command) {
        String name = command.name;
        double data = command.data;
        long time = System.currentTimeMillis() + (long)(command.time * 1000);

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
            }
        }

        int ticks = (int)(revs * ticksForMotors);

        switch(name) {
            case "MOVE":
                SetTicksAndMotorsForMovement(ticks, false);
                break;
            case "ROTATE":
                SetTicksAndMotorsForMovement(ticks, true);
                break;
            case "ONEREVROT":
                SetTicksAndMotorsForMovement(ticks, true);
                break;
        }

        commandsIndex++;
        runningAuto = false;
    }

    void SetTicksAndMotorsForMovement(int ticks, boolean rot) {
        int rotMultiplier = 1;
        if(rot) rotMultiplier = -1;

        left_back.setTargetPosition(ticks);
        left_front.setTargetPosition(rotMultiplier * ticks);
        right_back.setTargetPosition(rotMultiplier * ticks);
        right_front.setTargetPosition(ticks);
        RunToPositionAutonomousMovement();

        double ticksSpeed = MAX_NUM_TICKS_MOVEMENT * 0.5;

        left_back.setVelocity(ticksSpeed);
        left_front.setVelocity(rotMultiplier * ticksSpeed);
        right_back.setVelocity(rotMultiplier * ticksSpeed);
        right_front.setVelocity(ticksSpeed);

        while(left_back.isBusy() && opModeIsActive()) {

        }
    }

    public void ChildCommandInitialization() {}
}
