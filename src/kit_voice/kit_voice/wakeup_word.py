import os
import numpy as np
import sounddevice as sd
from openwakeword.model import Model
from ament_index_python.packages import get_package_share_directory

from kit_voice.audio_device import resolve_input_device

PACKAGE_NAME = "kit_voice"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

# 레퍼런스(이전 프로젝트) 웨이크워드 모델 그대로 이식한 임시값.
# "hello rokey" 발화 전용으로 학습된 모델이라 우리 프로젝트 웨이크워드가 정해지면 교체해야 한다.
MODEL_NAME = "hello_rokey_8332_32.tflite"
MODEL_PATH = os.path.join(PACKAGE_PATH, f"resource/{MODEL_NAME}")

SAMPLE_RATE = 16000
FRAME = 1280


class WakeupWord:
    def __init__(self):
        self.model = None
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0]
        self.stream = None

    def is_wakeup(self):
        audio_chunk, _ = self.stream.read(FRAME)
        audio_chunk = audio_chunk.flatten()
        confidence = self.model.predict(audio_chunk)[self.model_name]
        print("confidence: ", confidence)
        if confidence > 0.3:
            print("Wakeword detected!")
            return True
        return False

    def open(self):
        self.model = Model(wakeword_models=[MODEL_PATH])
        self.stream = sd.InputStream(
            samplerate=SAMPLE_RATE, channels=1, dtype="int16", blocksize=FRAME,
            device=resolve_input_device(),
        )
        self.stream.start()

    def close(self):
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
            self.stream = None
