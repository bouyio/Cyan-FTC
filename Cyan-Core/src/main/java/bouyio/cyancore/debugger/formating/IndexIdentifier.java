package bouyio.cyancore.debugger.formating;

public class IndexIdentifier implements StringIdentifiable {

    private final int index;

    public IndexIdentifier(int i) {
        index = i;
    }

    @Override
    public String getIdentifier() {
        return "Index[" + index + "]";
    }
}
