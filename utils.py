def make_ack(cmd: str) -> bytes:
    return f"ACK:{cmd}\n".encode()

