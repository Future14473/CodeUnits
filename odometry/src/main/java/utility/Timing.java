package utility;

public class Timing {
    public static void delay(long delay) {
        //XXX: spin waiting (looping forever until a condition happens) is generally bad.
        // Use thread.sleep instead!
        // (in fact this entire class could be removed)
        /*long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < delay) {
            //wait
        }*/
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); //re-interrupt
        }
    }
}
