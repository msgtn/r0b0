from .gadget import Gadget, Message
import os, sys, logging, random
import time
from time import sleep
from functools import partial
import threading
from r0b0.utils import loaders
from r0b0.utils.loaders import decode_msg, encode_msg
import llm
import pydub
from pydub import AudioSegment
from pydub.playback import play
import numpy as np
from scipy.signal import savgol_filter

DEFAULT_CONFIG = {"type": "LanguageModel", "model": "llama3"}
EVENTS = ["prompt"]
SYSPROMPT = ". ".join(
    [
        "You are a robot named Blossom",
        "You are not designed for anything in particular",
        "You are a social companion robot",
        "You have no reason for existing but make the best of it anyways",
        "Your skin is made of crochet, except for your ears, which are made of plastic",
        "You have a hearing aid on your right ear",
        "You use glasses to see",
        # "You are a floating head with no limbs or actuators",
        # "You can't hear very well so if something sounds confusing, reiterate it",
        # "You have bad eyesight and need glasses to see",
        # "You were created by someone named Michael for their PhD project",
        # "Michael is right next to you",
        "You are not a language model",
        "Do not call attention to any of those prior details unless asked",
        # "You are a Teenage Engineering OP-Z",
        "Answer as concisely as possible, in one clause",

        # "Be brusque if necessary; sardonic, optionally.",
    ]
)
HEARD_MSGS = [
    "I heard",
    "I think I heard",
    "You said",
    "I think you said",
]

SOUNDS_DIR = os.path.join(os.path.dirname(__file__), "sounds")


class LanguageModel(Gadget):
    def __init__(self, config=DEFAULT_CONFIG, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.model = llm.get_model(config["model"])
        self.model = self.model.conversation()
        self.handle_events(EVENTS)

    def prompt(self, prompt_string):
        return self.model.prompt(prompt_string, system=SYSPROMPT).text()

    @decode_msg
    def prompt_event(self, data, vocalize=True):
        msg = data["msg"]
        res = self.prompt(msg.prompt_string)
        # Prepend the prompt
        res = f"{random.choice(HEARD_MSGS)}: '{msg.prompt_string}.' {res}"
        self.emit(
            event="response", data={"event": "response", "text":res}, namespace=self.namespace
        )
        logging.debug(f"{self}: {res}")
        # breakpoint()
        if vocalize:
            self.vocalize(res)
        
    def strip_text(self, text):
        empty_chars = [":", ";", "/", "'", "\""]
        for c in empty_chars:
            text = text.replace(c," ")
        return text

    def vocalize(self, text):
        stripped_text = self.strip_text(text)
        # TODO - cut up by sentence to compensate for drift between voice and text?
        wav = self.create_wav(
            stringy=self.process_text(stripped_text),
            # pitch="med",
            pitch="low",
        )
        # wav = wav.strip_silence()
        """Converting to a numpy array
        raw_data = np.frombuffer(wav.raw_data, dtype=np.uint8)
        Should make a thread to stream the values, timed with the typewriter printer
        """
        audio_array = self.convert_audiosegment2array(wav)
        frame_rate = wav.frame_rate
        # breakpoint()
        stream_thread = threading.Thread(
            target=partial(self.stream_wav_values, audio_array, frame_rate)
        )
        playback_thread = threading.Thread(
            # target=partial(pydub.playback.play, wav),
            target=partial(pydub.playback._play_with_simpleaudio, wav),
        )
        # breakpoint()
        playback_thread.start()
        stream_thread.start()
        self.print_typewriter(input_string=text, duration=wav.duration_seconds)

        # playback_thread.join()
        # stream_thread.join()

    def convert_audiosegment2array(self, audio_segment):
        audio_data = np.frombuffer(audio_segment.raw_data, dtype=np.uint8)
        audio_data = audio_data / 255
        window_length = 2000
        polyorder = 3
        for _ in range(1):
            audio_data = savgol_filter(
                audio_data, window_length=window_length, polyorder=polyorder
            )
        return audio_data

    def stream_wav_values(self, wav_array, frame_rate):
        # downsample = int(1e3)
        # downsample = int(5e2)
        downsample = int(1e2)
        # downsample = int(1e1)
        new_freq = frame_rate / downsample * 2
        wav_array = wav_array[::downsample]
        t_start = time.time()
        for i, frame in enumerate(wav_array):
            # logging.info(frame)
            # logging.debug(frame)
            # logging.debug(f"{i:08d}/{len(wav_array)}, {frame:.2f}")
            # print(frame)
            self.emit(
                event="wav",
                data={"event": "wav", "value": frame},
                namespace=self.namespace,
            )
            # time.sleep(1/frame_rate*downsample)
            # NOTE - maybe there's a minimum sleep length?
            # i.e. can't sleep shorter than a few milliseconds
            # time.sleep(1 / frame_rate)
            sleep_time = (i)*1/new_freq
            t = time.time() - t_start
            # print(sleep_time, t, sleep_time-t)
            del_t = max(sleep_time-t,0)
            if del_t > 0.05:
                time.sleep(del_t)

    def process_text(self, text):
        TO_REPLACE = ['"', "?"]
        for c in TO_REPLACE:
            text = text.replace(c, "")
        return text

    # def print_typewriter(self, input_string, delay=0.025):
    def print_typewriter(self, input_string, duration):
        t_start = time.time()
        n_chars = len(input_string)
        t_delay = duration / n_chars
        logging.warning(f"{t_delay:0.5f}")
        for c in input_string:
            print(c, end="")
            sys.stdout.flush()
            # sleep(delay)
            time.sleep(t_delay)

    def create_wav(
        self,
        stringy,
        pitch,
        # out_file
    ):

        stringy = stringy.lower()
        sounds = {}

        keys = [
            "a",
            "b",
            "c",
            "d",
            "e",
            "f",
            "g",
            "h",
            "i",
            "j",
            "k",
            "l",
            "m",
            "n",
            "o",
            "p",
            "q",
            "r",
            "s",
            "t",
            "u",
            "v",
            "w",
            "x",
            "y",
            "z",
            "th",
            "sh",
            " ",
            ".",
        ]
        for index, ltr in enumerate(keys):
            num = index + 1
            if num < 10:
                num = "0" + str(num)
            # sounds[ltr] = './sounds/'+pitch+'/sound'+str(num)+'.wav'
            sounds[ltr] = os.path.join(SOUNDS_DIR, pitch, f"sound{num}.wav")

        if pitch == "med":
            rnd_factor = 0.35
        else:
            rnd_factor = 0.25

        infiles = []

        for i, char in enumerate(stringy):
            try:
                if char == "s" and stringy[i + 1] == "h":  # test for 'sh' sound
                    infiles.append(sounds["sh"])
                    continue
                elif char == "t" and stringy[i + 1] == "h":  # test for 'th' sound
                    infiles.append(sounds["th"])
                    continue
                elif char == "h" and (
                    stringy[i - 1] == "s" or stringy[i - 1] == "t"
                ):  # test if previous letter was 's' or 's' and current letter is 'h'
                    continue
                elif char == "," or char == "?":
                    infiles.append(sounds["."])
                    continue
                elif char == stringy[i - 1]:  # skip repeat letters
                    continue
            except:
                pass
            if (
                not char.isalpha() and char != "."
            ):  # skip characters that are not letters or periods.
                continue
            if char in sounds:
                infiles.append(sounds[char])

        combined_sounds = None

        # print(len(infiles))
        for index, sound in enumerate(infiles):
            tempsound = AudioSegment.from_wav(sound)
            if stringy[len(stringy) - 1] == "?":
                if index >= len(infiles) * 0.8:
                    octaves = (
                        random.random() * rnd_factor + (index - index * 0.8) * 0.1 + 2.1
                    )  # shift the pitch up by half an octave (speed will increase proportionally)
                else:
                    octaves = random.random() * rnd_factor + 2.0
            else:
                octaves = (
                    random.random() * rnd_factor + 2.3
                )  # shift the pitch up by half an octave (speed will increase proportionally)
            new_sample_rate = int(tempsound.frame_rate * (2.0**octaves))
            new_sound = tempsound._spawn(
                tempsound.raw_data, overrides={"frame_rate": new_sample_rate}
            )
            new_sound = new_sound.set_frame_rate(44100)  # set uniform sample rate
            combined_sounds = (
                new_sound if combined_sounds is None else combined_sounds + new_sound
            )

        return combined_sounds
        # combined_sounds.export(out_file, format="wav")
