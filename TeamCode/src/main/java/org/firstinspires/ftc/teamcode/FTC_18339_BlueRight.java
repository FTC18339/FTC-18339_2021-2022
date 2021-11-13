package org.firstinspires.ftc.teamcode;

public class FTC_18339_BlueRight extends FTC_18339_AutonomousProtocol001 {
    @Override
    public void ChildCommandInitialization(){
        commands = new Command[] {
                new Command("ROTATE", 90, true, 0.0f),
                new Command("MOVE", 24 * mmPerInch, true, 0.0f)
        };
    }
}
