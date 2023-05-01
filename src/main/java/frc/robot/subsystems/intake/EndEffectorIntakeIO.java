package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;

public interface EndEffectorIntakeIO {
    public static class EndEffectorIntakeIOInputs{
        public boolean storing = false;
        public double appliedSpeed = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;

        public void toLog(LogTable table){
            table.put("Storing?", storing);
            table.put("Outtake Speed", appliedSpeed);
            table.put("Voltage", appliedVolts);
            table.put("Current", current);
        }

        public void fromLog(LogTable table){
            table.getBoolean("Storing?", storing);
            table.put("Outtake Speed", appliedSpeed);
            table.put("Voltage", appliedVolts);
            table.put("Current", current);
        }
    }

    public default void updateInputs(EndEffectorIntakeIOInputs inputs) {}

    public default void setAppliedSpeed(double speed){}

}
