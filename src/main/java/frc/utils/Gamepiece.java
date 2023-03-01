package frc.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gamepiece {
    private static GamepieceType gamepiece = GamepieceType.None;

    public static enum GamepieceType {
        Cube, Cone, None;
    }

    public static void pickCube() {
        SmartDashboard.putString("Gamepiece", gamepiece.name());
        gamepiece = GamepieceType.Cube;
    }

    public static void pickCone() {
        SmartDashboard.putString("Gamepiece", gamepiece.name());
        gamepiece = GamepieceType.Cone;
    }

    public static GamepieceType getGamepiece() {
        return gamepiece;
    }
}