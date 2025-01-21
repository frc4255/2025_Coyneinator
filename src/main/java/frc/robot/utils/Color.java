package frc.robot.utils;

public class Color {
    public int RED;
    public int GREEN;
    public int BLUE;
    public int WHITE;
    public int startIDx;
    public int count;
    public boolean STROBE;
    public double SPEED;
    

    public Color() {
        this.RED = 0;
        this.GREEN = 0;
        this.BLUE = 0;
        
    }

    public Color(int RED, int GREEN, int BLUE) {
        this.RED = RED;
        this.GREEN = GREEN;
        this.BLUE = BLUE;
    }

    public Color(int RED, int GREEN, int BLUE, int WHITE) {
        this.RED = RED;
        this.GREEN = GREEN;
        this.BLUE = BLUE;
        this.WHITE = WHITE;
    }

    public Color(int RED, int GREEN , int BLUE, boolean STROBE, double SPEED) {
        this.RED = RED;
        this.GREEN = GREEN;
        this.BLUE = BLUE;
        this.STROBE = STROBE;
        this.SPEED = SPEED;
    }

}

