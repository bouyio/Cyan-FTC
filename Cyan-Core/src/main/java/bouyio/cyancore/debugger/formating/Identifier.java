package bouyio.cyancore.debugger.formating;

public class Identifier implements StringIdentifiable {

    String identifier;

    public Identifier(String id) {
        identifier = id;
    }

    @Override
    public String getIdentifier() {
        return identifier;
    }
}
