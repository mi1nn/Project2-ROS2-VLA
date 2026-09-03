import os

import sounddevice as sd


def resolve_input_device():

    # ① 환경변수에서 마이크를 직접 지정했는지 확인
    override = os.environ.get("VOICE_MIC_DEVICE")

    if override:
        return int(override) if override.isdigit() else override

    # ② 직접 지정하지 않았다면 장치를 하나씩 찾아봄
    try:
        for idx, dev in enumerate(sd.query_devices()):

            # ③ 마이크이고 16kHz인 장치를 찾음
            if dev["max_input_channels"] > 0 \
                    and int(round(dev["default_samplerate"])) == 16000:

                # ④ 찾으면 장치 번호 반환
                return idx

    except Exception:
        pass

    # ⑤ 아무것도 못 찾으면
    return None