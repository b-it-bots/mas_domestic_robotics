from StringIO import StringIO


def to_cpp(msg):
    buf = StringIO()
    msg.serialize(buf)
    return buf.getvalue()


def from_cpp(serial_msg, cls):
    msg = cls()
    return msg.deserialize(serial_msg)
