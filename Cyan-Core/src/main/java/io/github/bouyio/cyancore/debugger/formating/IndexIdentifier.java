package io.github.bouyio.cyancore.debugger.formating;

/**
 * <p>An identifier that stores the index of an iterable object.<p/>
 * @see StringIdentifiable
 * */
public class IndexIdentifier implements StringIdentifiable {

    private final int index;

    /**
     * <p>Creates an index identifier object with a specified index.<p/>
     * */
    public IndexIdentifier(int i) {
        index = i;
    }

    /**
     * <p>Returns the formated index as Index[{@code SPECIFIED_INDEX}]. <p/>
     * */
    @Override
    public String getIdentifier() {
        return "Index[" + index + "]";
    }
}
