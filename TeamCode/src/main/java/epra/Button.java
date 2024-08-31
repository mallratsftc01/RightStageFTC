package epra;

//Queer coded - Zachy K
public class Button {

    private float value;

    public boolean flag;
    public boolean toggle;
    public int counter;

    /**Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * <p></p>
     * A storage class for button or analog data.
     * @param value A starting float value
     */
    public Button(float value) {
        this.value = value;
        flag = false;
        toggle = false;
        counter = 0;
    }
    /**Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * <p></p>
     * A storage class for button or analog data.
     * @param value A starting boolean value
     */
    public Button(boolean value) {
        this.value = (value) ? 1.0f : 0.0f;
        flag = false;
        toggle = false;
        counter = 0;
    }
    /**Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * <p></p>
     * A storage class for button or analog data.
     */
    public Button() {
        value = 0.0f;
        flag = false;
        toggle = false;
        counter = 0;
    }

    /**@param value Updates the button value to this float value.*/
    public void update(float value) { this.value = value; }
    /**@param value Updates the button value to this boolean value.*/
    public void update(boolean value) { this.value = (value) ? 1.0f : 0.0f; }

    /**Returns the button's value as a float.*/
    public float toFloat() { return value; }
    /**Returns the button's value as a float.*/
    public boolean toBoolean() { return (value != 0.0f); }
}
