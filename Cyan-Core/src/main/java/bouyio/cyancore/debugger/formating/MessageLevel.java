package bouyio.cyancore.debugger.formating;

public enum MessageLevel implements StringIdentifiable  {
    DEBUG("Debug"),
    INFO("Info"),
    WARNING("Warning"),
    ERROR("Error"),
    CRITICAL("Critical");

    final String identifier;

    MessageLevel(String id) {
        identifier = id;
    }

    @Override
    public String getIdentifier() {
        return identifier;
    }
}
