import os
import time
import select

DEVICE = "/dev/ttyebus"

def simulate():
    try:
        # Öffnen im Non-Blocking Modus
        fd = os.open(DEVICE, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        
        # 1. 'SYN' simulieren (0xAA), damit der Bus "lebt"
        print("Sende Synchronisations-Bytes (0xAA)...")
        os.write(fd, b'\xaa\xaa\xaa')
        time.sleep(0.05)
        
        # 2. Eine Test-Nachricht senden
        test_msg = b'\x03\x05\x07\x00'
        print(f"Sende Nachricht: {test_msg.hex(' ')}")
        os.write(fd, test_msg)
        
        # 3. Auf Antwort warten mit Timeout (select)
        print("Warte auf Echo/Antwort (2 Sekunden)...")
        ready = select.select([fd], [], [], 2.0)
        
        if ready[0]:
            data = os.read(fd, 128)
            print(f"Empfangen: {data.hex(' ')}")
        else:
            print("Timeout: Keine Daten empfangen. Ist der RX-Pin verbunden?")
            
        os.close(fd)
    except Exception as e:
        print(f"Fehler: {e}")

if __name__ == "__main__":
    simulate()
