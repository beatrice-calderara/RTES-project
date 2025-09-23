import torch
import librosa
from transformers import Wav2Vec2ForSequenceClassification, Wav2Vec2FeatureExtractor

# Modello addestrato sul dataset SpeechCommands
MODEL_NAME = "superb/wav2vec2-base-superb-ks"

# Carico feature extractor e modello
processor = Wav2Vec2FeatureExtractor.from_pretrained(MODEL_NAME)
model = Wav2Vec2ForSequenceClassification.from_pretrained(MODEL_NAME)

def load_audio(path, target_sampling_rate=16000):
    """
    Carica un file audio (wav/mp3) e lo riporta al sample rate richiesto dal modello.
    """
    audio, sr = librosa.load(path, sr=target_sampling_rate)
    return audio, sr

def classify_keyword(audio_path):
    """
    Classifica l'audio in una delle parole previste dal modello.
    Stampa solo la predizione con la probabilità più alta.
    """
    # Carico audio
    audio, sr = load_audio(audio_path)

    # Estraggo feature
    inputs = processor(audio, sampling_rate=sr, return_tensors="pt", padding=True)

    # Inference senza gradiente
    with torch.no_grad():
        logits = model(**inputs).logits

    # Softmax → probabilità
    probs = torch.nn.functional.softmax(logits, dim=-1)

    # Indice della predizione con probabilità più alta
    predicted_id = torch.argmax(probs, dim=-1).item()

    # Nome della label corrispondente
    predicted_label = model.config.id2label[predicted_id]
    predicted_score = probs[0][predicted_id].item()

    print(f"Parola riconosciuta: {predicted_label} ({predicted_score:.2f})")

    return predicted_label

def handle_command(audio_path):
    """
    Riconosce se l'audio contiene una parola tra quelle valide.
    Se sì → salva in variabile e stampa.
    Se no → stampa 'nessuna parola riconosciuta'.
    """
    command = classify_keyword(audio_path)

    valid_commands = {"on", "off", "stop", "go"}
    recognized_command = None  # variabile dove salvare il risultato

    if command in valid_commands:
        recognized_command = command
        print(f"Comando riconosciuto: {recognized_command.upper()}")
    else:
        print("Nessuna parola riconosciuta")

    return recognized_command


if __name__ == "__main__":
    audio_file = "audio_off.m4a"
    classify_keyword(audio_file)

