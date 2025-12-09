import serial
import numpy as np
import torch
from transformers import Wav2Vec2ForSequenceClassification, Wav2Vec2FeatureExtractor
from collections import deque
import time
import threading
import queue


PORT = '/dev/cu.usbmodem11101'
BAUD = 115200

# Marker for audio protocol
MARKER_START = 0xF000
MARKER_END = 0xE000
VALUE_MASK = 0x0FFF

BUFFER_SIZE = 256
SAMPLE_RATE = 16000

# Parameters for keyword spotting
WINDOW_SIZE = 16000
INFERENCE_INTERVAL = 0.4
CONFIDENCE_THRESHOLD = 0.85
RMS_THRESHOLD = 60

# Commands mapping
COMMAND_MAP = {
    'yes': 'on', 'no': 'off', 'up': 'blink', 'down': 'alarm',
    'on': 'on', 'off': 'off',
}



class KeywordSpotter:
    def __init__(self, model_name="superb/wav2vec2-base-superb-ks"):
        print(f"Caricamento modello {model_name}...")
        try:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.model = Wav2Vec2ForSequenceClassification.from_pretrained(model_name).to(self.device)
            self.feature_extractor = Wav2Vec2FeatureExtractor.from_pretrained(model_name)
            self.model.eval()
            self.id2label = self.model.config.id2label
            print(f"Device: {self.device}")
            print(f"Classi disponibili: {list(self.id2label.values())}")
        except Exception as e:
            print(f"[ERRORE CRITICO] Impossibile caricare il modello: {e}")
            exit(1)  # Exit if the model can't be loaded

        self.packet_queue = queue.Queue(maxsize=100)
        self.ser = None
        self.running = False
        self.packets_received = 0
        self.commands_sent = 0
        self.last_command_time = 0
        self.command_cooldown = 3.0

    def connect_serial(self):
        # Connect to the serial port
        try:
            print(f"Connessione a {PORT}...")
            self.ser = serial.Serial(PORT, BAUD, timeout=1.0)  # Timeout increased of 1 second
            time.sleep(2)
            if self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)
            print("✓ Connesso")
            return True
        except serial.SerialException as e:
            print(f"[ERRORE] Impossibile connettersi alla porta {PORT}.")
            print(f"  Dettagli: {e}")
            return False


    def read_packet(self):
        # Read one complete audio packet from the Pico, it synchronizes byte per byte
        try:
            while self.running:
                # Search for the first byte (LSB) of a possible packet
                byte1 = self.ser.read(1)
                if not byte1:
                    # print("[DEBUG] Timeout in attesa del primo byte.")
                    continue

                # Read second byte (MSB)
                byte2 = self.ser.read(1)
                if not byte2:
                    # print("[DEBUG] Timeout in attesa del secondo byte.")
                    continue

                # Reconstruct the header in little-endian
                header = byte1[0] | (byte2[0] << 8)

                # Check if the marker is a START marker
                if (header & 0xF000) == MARKER_START:
                    # Found, let's read the rest of the packet
                    # print(f"[DEBUG] ✓ START marker trovato (0x{header:04X})!")

                    # The packet is composed by BUFFER_SIZE sample (uint16), so BUFFER_SIZE * 2 byte in tot
                    # We read the first 2, so the remaining are (BUFFER_SIZE - 1) * 2
                    payload_size = (BUFFER_SIZE - 1) * 2
                    payload_bytes = self.ser.read(payload_size)

                    if len(payload_bytes) < payload_size:
                        print(f"[DEBUG] ✗ ERRORE: Payload incompleto! Letti {len(payload_bytes)}/{payload_size} byte.")
                        continue  # Discard and search another START

                    # Reconstruct and validate the complete packet
                    full_packet_bytes = byte1 + byte2 + payload_bytes
                    samples = np.frombuffer(full_packet_bytes, dtype=np.uint16)

                    # Check for the END marker
                    if (samples[-1] & 0xF000) == MARKER_END:
                        # print("[DEBUG] ✓ END marker valido. Pacchetto OK.")
                        self.packets_received += 1
                        return samples & VALUE_MASK  # Valid packet, return clean data
                    else:
                        print(f"[DEBUG] ✗ ERRORE: END marker non valido! Trovato: 0x{samples[-1]:04X}")
                        continue  # Discard and restart

                # print(f"[DEBUG] Header non valido: 0x{header:04X}. In cerca del prossimo...")

        except serial.SerialException as e:
            print(f"[ERRORE] Eccezione nella lettura seriale: {e}")
            self.running = False
            return None

    def send_command(self, command):
        # Send a command to the Pico with cooldown logics
        current_time = time.time()
        if (current_time - self.last_command_time) < self.command_cooldown:
            # print(f"  Comando '{command.upper()}' ignorato (cooldown attivo).")
            return

        try:
            msg = f"CMD:{command}\n"
            self.ser.write(msg.encode('utf-8'))
            self.last_command_time = current_time
            self.commands_sent += 1
            print(f"→ Comando inviato: {command.upper()}")
        except serial.SerialException:
            print("[ERRORE] Impossibile inviare il comando (porta chiusa).")

    def predict_keyword(self, audio_window):
        # Perform inference on the audio buffer
        audio_normalized = audio_window / 2048.0
        inputs = self.feature_extractor(audio_normalized, sampling_rate=SAMPLE_RATE, return_tensors="pt")
        with torch.no_grad():
            logits = self.model(inputs.input_values.to(self.device)).logits
        probabilities = torch.nn.functional.softmax(logits, dim=-1)
        confidence, predicted_id = torch.max(probabilities, dim=-1)
        return self.id2label[predicted_id.item()], confidence.item()


    def audio_capture_thread(self):
        # Producer: reads packets from the Pico and put them in the queue
        print("[THREAD] Audio capture (Produttore) avviato.")
        while self.running:
            packet = self.read_packet()
            if packet is not None:
                try:
                    self.packet_queue.put_nowait(packet)
                except queue.Full:
                    # The queue is full, discard the oldest packet
                    self.packet_queue.get_nowait()
                    self.packet_queue.put_nowait(packet)
            else:
                if self.running: print("[WARN] Pacchetto audio perso o corrotto.")
        print("[THREAD] Audio capture terminato.")

    def inference_thread(self):
        # Consumer: takes the packets from the queue and perform inference
        print("[THREAD] Inference (Consumatore) avviato.")
        audio_buffer = deque(maxlen=WINDOW_SIZE)
        audio_buffer.extend(np.zeros(WINDOW_SIZE))

        last_print_time = time.time()

        while self.running:
            try:
                # Empty the queue to consider only recent data
                while not self.packet_queue.empty():
                    packet = self.packet_queue.get_nowait()
                    # Samples are uint16, the "zero" value is 2048
                    audio = np.array(packet, dtype=np.int16) - 2048
                    audio_buffer.extend(audio)

                # Create a copy for the analysis
                audio_window = np.array(audio_buffer, dtype=np.float32)
                rms = np.sqrt(np.mean(audio_window ** 2))

                # Print RMS every second to monitor volume
                current_time = time.time()
                if current_time - last_print_time > 1.0:
                    print(f"[DEBUG] RMS attuale: {rms:.2f} (Soglia per inferenza: {RMS_THRESHOLD})")
                    last_print_time = current_time

                # Perform inference only if there's the signal
                if rms > RMS_THRESHOLD:
                    print(f"-> [INFERENZA] RMS sufficiente ({rms:.2f})! Avvio analisi...")
                    label, confidence = self.predict_keyword(audio_window)

                    # Always print inference result
                    print(
                        f"  --> Predizione: '{label}' (Confidenza: {confidence:.2f}, Soglia comando: {CONFIDENCE_THRESHOLD})")

                    if confidence > CONFIDENCE_THRESHOLD:
                        command = COMMAND_MAP.get(label.lower())
                        if command:
                            print(
                                f"✓✓✓ Rilevato '{label}' con confidenza sufficiente → Invia comando {command.upper()}")
                            self.send_command(command)
                            # Clean the buffer
                            audio_buffer.clear()
                            audio_buffer.extend(np.zeros(WINDOW_SIZE))

                time.sleep(INFERENCE_INTERVAL)

            except Exception as e:
                print(f"[ERRORE] Errore critico nel thread di inferenza: {e}")
                import traceback
                traceback.print_exc()

        print("[THREAD] Inference terminato.")

    def stats_thread(self):
        # Print statistics
        while self.running:
            time.sleep(5)
            qsize = self.packet_queue.qsize()
            print(
                f"\n[STATS] Pacchetti Ricevuti: {self.packets_received} | Comandi Inviati: {self.commands_sent} | Coda: {qsize}\n")
        print("[THREAD] Stats terminato.")

    def run(self):
        # Start the system
        if not self.connect_serial():
            return

        self.running = True

        threads = [
            threading.Thread(target=self.audio_capture_thread, daemon=True),
            threading.Thread(target=self.inference_thread, daemon=True),
            threading.Thread(target=self.stats_thread, daemon=True)
        ]

        for t in threads: t.start()

        print("\n" + "=" * 50 + "\nSistema keyword spotting avviato (v2)\n" + "=" * 50 + "\n")

        try:
            while self.running:
                if not all(t.is_alive() for t in threads):
                    print("[ERRORE] Un thread si è interrotto. Chiusura del sistema.")
                    self.running = False
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\nInterruzione manuale...")
        finally:
            self.running = False
            print("In attesa della chiusura dei thread...")
            time.sleep(1.5)
            if self.ser and self.ser.is_open:
                self.ser.close()
            print("✓ Sistema chiuso correttamente.")


if __name__ == "__main__":
    spotter = KeywordSpotter()
    spotter.run()
