package com.github.bouyio.cyancore.debugger.formating;

/**
 * <p>A simple identifier that is compatible with StringIdentifiable.<p/>
 * @see StringIdentifiable
 * */
public class Identifier implements StringIdentifiable {

    String identifier;

    /**
     * <p>Creates an identifier with a specified value.<p/>
     * */
    public Identifier(String id) {
        identifier = id;
    }

    /**
     * <p>Returns the stored identifier.<p/>
     * */
    @Override
    public String getIdentifier() {
        return identifier;
    }
}
