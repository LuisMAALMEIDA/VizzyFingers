package C;

public interface DependsOnB extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "C/DependsOnB";
  static final java.lang.String _DEFINITION = "std_msgs/String stringProperty\nB/Standalone dependentProperty ";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.String getStringProperty();
  void setStringProperty(std_msgs.String value);
  B.Standalone getDependentProperty();
  void setDependentProperty(B.Standalone value);
}
