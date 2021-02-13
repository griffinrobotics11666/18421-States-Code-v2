package org.firstinspires.ftc.teamcode.util;

/**
 * A class that cycles through different values according to the number of values specified.
 */
public class Cycle {
    private int value = 1;
    private int numValues;

    /**
     * Constructor for the Cycle class.
     * @param numCycles the number of values the cycle has.
     * @return values from {@code 1} to {@code numCycles}. If the value is already {@code numCycles}, it becomes {@code 1}.
     */
    public Cycle(int numCycles){
        numValues = numCycles;
    }
    /**
     * Constructor for the Cycle class.
     * @param numCycles the number of values the cycle has.
     * @param value the value to start with. Value must be between 0 and {@code numCycles-1}.
     * @return values from {@code 1} to {@code numCycles}. If the value is already {@code numCycles}, it becomes {@code 1}.
     */
    public Cycle(int numCycles, int value){
        numValues = numCycles;
        if(value < 1 || value > numValues) {
            this.value = 1;
        }
        else {
            this.value = value;
        }
    }

    /**
     * cycles this {@link Cycle} between {@code 1} and {@code numCycles}. If the value is already {@code numCycles}, it becomes {@code 1}.
     */
    public void cycle(){
        value++;
        if(value > numValues){
            value = 1;
        }
    }

    public int numValues(){
        return numValues;
    }
    public int getValue(){
        return value;
    }

    public void setValue(int value){
        if(value < 1 || value > numValues) {

        }
        else {
            this.value = value;
        }
    }
}
