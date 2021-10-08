package org.firstinspires.ftc.teamcode;

public class Command {

    public String name;
    public float data;
    public boolean positional;
    public float time;

    public Command(String name_, float data_, boolean positional_, float time_) {
        name = name_;
        data = data_;
        positional = positional_;
        time = time_;
    }
}
