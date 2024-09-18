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

DEFAULT_CONFIG = {
    "type": "LanguageModel", 
    "model": "llama3"
}
EVENTS = ["prompt"]
SYSPROMPT = """
Answer as concisely as possible, in one sentence.
Be brusque if necessary; sardonic, optionally.
"""

SOUNDS_DIR = os.path.join(os.path.dirname(__file__), "sounds")

class LanguageModel(Gadget):
    def __init__(self, config=DEFAULT_CONFIG, **kwargs):
        Gadget.__init__(self , config, **kwargs)
        self.model = llm.get_model(config["model"])
        self.model = self.model.conversation()
        self.handle_events(EVENTS)

    def prompt(self, prompt_string):
        return self.model.prompt(
            prompt_string,
            system=SYSPROMPT).text()
    
    @decode_msg
    def prompt_event(self, data, vocalize=True):
        msg = data["msg"]
        res = self.prompt(msg.prompt_string)
        if vocalize:
            self.vocalize(res)

    def vocalize(self, text):
        # TODO - cut up by sentence to compensate for drift between voice and text?
        wav = self.create_wav(
            stringy=self.process_text(text),
            # pitch="med",
            pitch="low",
        )
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
            target=partial(pydub.playback.play, wav),
        )
        # breakpoint()
        playback_thread.start()
        stream_thread.start()
        self.print_typewriter(input_string=text)

        playback_thread.join()
        stream_thread.join()

    def convert_audiosegment2array(self, audio_segment):
        audio_data = np.frombuffer(audio_segment.raw_data, dtype=np.uint8)
        audio_data = audio_data / 255
        window_length = 2000
        polyorder = 3
        audio_filtered = savgol_filter(audio_data, window_length=window_length, polyorder=polyorder)
        return audio_filtered

    def stream_wav_values(self, wav_array, frame_rate):
        downsample = 1000
        wav_array = wav_array[::downsample]
        for i,frame in enumerate(wav_array):
            # logging.info(frame)
            # logging.debug(frame)
            logging.debug(f"{i:08d}/{len(wav_array)}, {frame:.2f}")
            # print(frame)
            self.emit(
                event="wav",
                data={"event": "wav", "value": frame}
            )
            # time.sleep(1/frame_rate*downsample)
            # NOTE - maybe there's a minimum sleep length?
            # i.e. can't sleep shorter than a few milliseconds
            time.sleep(1/frame_rate)
    
    def process_text(self, text):
        TO_REPLACE = ["\"", "?"]
        for c in TO_REPLACE:
            text = text.replace(c, "")
        return text
    
    def print_typewriter(self, input_string, delay=0.025):
        for c in input_string:
            print(c,end='')
            sys.stdout.flush()
            sleep(delay)
    
    def create_wav(self, stringy, pitch,
        # out_file
        ):

        stringy = stringy.lower()
        sounds = {}

        keys = ['a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','th','sh',' ','.']
        for index,ltr in enumerate(keys):
            num = index+1
            if num < 10:
                num = '0'+str(num)
            # sounds[ltr] = './sounds/'+pitch+'/sound'+str(num)+'.wav'
            sounds[ltr] = os.path.join(SOUNDS_DIR, pitch, f"sound{num}.wav")

        if pitch == 'med':
            rnd_factor = .35
        else:
            rnd_factor = .25

        infiles = []

        for i, char in enumerate(stringy):
            try:
                if char == 's' and stringy[i+1] == 'h': #test for 'sh' sound
                    infiles.append(sounds['sh'])
                    continue
                elif char == 't' and stringy[i+1] == 'h': #test for 'th' sound
                    infiles.append(sounds['th'])
                    continue
                elif char == 'h' and (stringy[i-1] == 's' or stringy[i-1] == 't'): #test if previous letter was 's' or 's' and current letter is 'h'
                    continue
                elif char == ',' or char == '?':
                    infiles.append(sounds['.'])
                    continue
                elif char == stringy[i-1]: #skip repeat letters
                    continue
            except:
                pass
            if not char.isalpha() and char != '.': # skip characters that are not letters or periods. 
                continue
            infiles.append(sounds[char])

        combined_sounds = None

        print(len(infiles))
        for index,sound in enumerate(infiles):
            tempsound = AudioSegment.from_wav(sound)
            if stringy[len(stringy)-1] == '?':
                if index >= len(infiles)*.8:
                    octaves = random.random() * rnd_factor + (index-index*.8) * .1 + 2.1 # shift the pitch up by half an octave (speed will increase proportionally)
                else:
                    octaves = random.random() * rnd_factor + 2.0
            else:
                octaves = random.random() * rnd_factor + 2.3 # shift the pitch up by half an octave (speed will increase proportionally)
            new_sample_rate = int(tempsound.frame_rate * (2.0 ** octaves))
            new_sound = tempsound._spawn(tempsound.raw_data, overrides={'frame_rate': new_sample_rate})
            new_sound = new_sound.set_frame_rate(44100) # set uniform sample rate
            combined_sounds = new_sound if combined_sounds is None else combined_sounds + new_sound


        return combined_sounds
        # combined_sounds.export(out_file, format="wav") 