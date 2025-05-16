package com.github.bouyio.cyancore.debugger.formating;

/**
 * <p>Some pre-made identifiers meant to be used as generic debug message headers.<p/>
 * @see StringIdentifiable
 * */
public enum MessageLevel implements StringIdentifiable  {
    DEBUG("Debug"),
    INFO("Info"),
    WARNING("Warning"),
    ERROR("Error"),
    CRITICAL("Critical");

    final String identifier;

    /**
     * <p>Registers the message level with a specified id.<p/>
     * */
    MessageLevel(String id) {
        identifier = id;
    }

    /**
     * <p>Returns the stored id.<p/>
     * */
    @Override
    public String getIdentifier() {
        return identifier;
    }
}
