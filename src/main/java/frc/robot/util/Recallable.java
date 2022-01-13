package frc.robot.util;

public interface Recallable<T> {
    public T recall();

    public CommandType getType();

    public enum CommandType {
        ROTATION,
        DISTANCE
    }

    public enum RecallMethod {
        REVERSE,
        REPEAT
    }
}