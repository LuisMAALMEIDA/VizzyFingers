package B;

public interface Standalone extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "B/Standalone";
  static final java.lang.String _DEFINITION = "uint32 intProperty\nstring stringProperty\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getIntProperty();
  void setIntProperty(int value);
  java.lang.String getStringProperty();
  void setStringProperty(java.lang.String value);
}
