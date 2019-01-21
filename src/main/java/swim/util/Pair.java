package swim.util;

/**
 * Source:
 * http://stackoverflow.com/questions/6271731/whats-the-best-way-to-return-a-pair-of-values-in-java
 */
 public class Pair<Key, Val> {         
    public final Key key;
    public final Val val;

    public Pair(Key key, Val val) {         
        this.key= key;
        this.val= val;
     }
 }
