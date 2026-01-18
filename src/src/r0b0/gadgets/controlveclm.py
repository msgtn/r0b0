from r0b0.gadgets.language_model import *
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer, AutoConfig
from repeng import ControlModel, ControlVector
import re

EVENTS.append("vector")
PARAMS =    [
    "vector", "temperature", "max_len", "repetition_penalty"
]
EVENTS.extend(PARAMS)

class ControlVectorLanguageModel(LanguageModel):
    def __init__(self, config=DEFAULT_CONFIG, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        MODEL_PATH = config["model_path"]
        TOKENIZER_PATH = config["tokenizer_path"]
        VECTOR_PATH = config["vector_path"]
        STATEDICT_PATH = config["statedict_path"]
        CONFIG_PATH = config["config_path"]
        _model = AutoModelForCausalLM.from_pretrained(
            MODEL_PATH,
            torch_dtype=torch.float16,
            config=AutoConfig.from_pretrained(CONFIG_PATH)
            # device_map="auto"
        )
        _model.to("mps")
        _model = self.load_nested_state_dict(_model, torch.load(STATEDICT_PATH))
        self.model = ControlModel(_model, list(range(-1, -10, -1)))

        self.tokenizer = AutoTokenizer.from_pretrained(TOKENIZER_PATH)
        self.tokenizer.pad_token_id = 0
        self.vector = ControlVector.import_gguf(VECTOR_PATH)
        self.vector_value = 1
        self.temperature_value = 0.7
        self.max_len_value = 20
        self.repetition_penalty_value = 1.1

        for param in PARAMS:
            setattr(self, f"{param}_event", partial(self.value_event, value_name=param))
        self.handle_events(EVENTS)


    def prompt(self, prompt_string, **kwargs):
        # breakpoint()
        # prompt_string = "I'm just testing this out"
        prompt_string = self.chat_template_unparse([("user", prompt_string)])
        ret = self.generate_with_vector(
            prompt_string,
            [(f"{self.vector_value} * vector", self.vector_value * self.vector)],
            **kwargs,
        )
        ret = self.tokenizer.batch_decode(ret[0])
        ret = self.chat_template_parse(ret[0])[-1][1]
        # logging.warning(f"CONTROL VEC LM: {ret}")
        return ret

    # @decode_msg
    # def vector_event(self, data):
    #     msg = data['msg']
    #     self.vector_value = msg.value
    #     logging.warning(f"Sarcasm level: {self.vector_value:0.3f}")
    #     # breakpoint()

    @decode_msg
    def value_event(self, data, value_name):
        msg = data["msg"]
        value_str = f"{value_name}_value"
        if hasattr(self, value_str):

            setattr(self, value_str, msg.value)
            if value_name == "vector":
                value_name = "sarcasm"
            logging.warning(f"{value_name.capitalize()} value: {msg.value:0.3f}")



    def chat_template_unparse(self, messages: list[tuple[str, str]]) -> str:
        template = []
        for role, content in messages:
            template.append(
                f"<|start_header_id|>{role}<|end_header_id|>\n\n{content}<|eot_id|>"
            )
        if messages[-1][0] != "assistant":
            # prefill assistant prefix
            template.append("<|start_header_id|>assistant<|end_header_id|>\n\n")
        return "".join(template)

    def chat_template_parse(self, resp: str) -> list[tuple[str, str]]:
        resp = resp.strip().removeprefix("<|begin_of_text|>")
        messages = []
        for part in resp.split("<|start_header_id|>"):
            role_and_content = part.split("<|end_header_id|>")
            if len(role_and_content) == 1:
                role, content = role_and_content[0], ""
            else:
                role, content = role_and_content
            content = content.split("<|eot_id|>")[0]
            messages.append((role.strip(), content.strip()))
        return messages

    def generate_with_vector(
        self,
        prompt: str,
        labeled_vectors: list[tuple[str, ControlVector]],
        # max_new_tokens: int = 128,
        # max_new_tokens: int = 30,
        # repetition_penalty: float = 1.1,
        show_baseline: bool = False,
        # temperature: float = 0.7,
    ):
        max_new_tokens = self.max_len_value
        temperature = self.temperature_value
        repetition_penalty = self.repetition_penalty_value
        logging.warning("PROMPT")
        # input_ids = tokenizer(input, return_tensors="pt").to("cuda:0")
        input_ids = self.tokenizer(prompt, return_tensors="pt").to("mps:0")
        settings = {
            "pad_token_id": self.tokenizer.eos_token_id,  # silence warning
            # "do_sample": False, # temperature=0
            "temperature": temperature,
            "max_new_tokens": max_new_tokens,
            "repetition_penalty": repetition_penalty,
        }
        rets = []

        def gen(label):
            # display(HTML(f"<h3>{label}</h3>"))
            # _ = self.model.generate(streamer=HTMLStreamer(tokenizer), **input_ids, **settings)
            response = self.model.generate(**input_ids, **settings)
            rets.append(response)
            # logging.warning(response)

        if show_baseline:
            self.model.reset()
            gen("baseline")
        for label, vector in labeled_vectors:
            self.model.set_control(vector)
            gen(label)
        self.model.reset()
        # logging.warning("RETS")
        # logging.warning(rets)
        return rets

    def load_nested_state_dict(self, _loaded_model, loaded_state_dict):
        new_state_dict = {}
        _loaded_state_dict = _loaded_model.state_dict()
        for key, value in _loaded_state_dict.items():
            new_key = key.replace("model.model.", "")
            while new_key not in _loaded_state_dict:
                new_key = f"model.{new_key}"
            old_key = new_key
            # if old_key not in loaded_state_dict:
            #     old_key = f"model.{new_key}"
            # if old_key not in loaded_state_dict:
            #     old_key = old_key.replace('.self_attn','.block.self_attn')
            # if old_key not in loaded_state_dict:

            #     old_key = old_key.replace('.self_attn','.block.self_attn')

            if old_key not in loaded_state_dict:
                # old_key = old_key.replace('.input_layernorm','.block.input_layernorm')
                old_key = re.sub(r"(\d+).",r"\1.block.",old_key)
                # breakpoint()
            old_value = loaded_state_dict[old_key]
            # print(torch.max(torch.abs(old_value - value)))
 
            new_state_dict.update({
                # new_key: value,
                new_key: old_value,
                })

        _loaded_model.load_state_dict(new_state_dict)
        return _loaded_model
