var dataSet = {
    "a": "testing a",
    "b": "testing b"
};
// export { dataSet };


var promptDict = {
  "happy1":"Your best friends are having a reunion and invited you.",
  "sad1":"Your best friends are having a reunion and accidentally forgot to invite you.",
  "angry1":"Your best friends are having a reunion and intentionally did not invite you.",
  "happy2":"Your proposed project idea was unanimously accepted by your colleagues.",
  "sad2":"Your proposed project idea was rejected, fairly, by your colleagues.",
  "angry2":"Your proposed project idea was stolen by a coworker and unanimously accepted by your colleagues.",
  "happy3":"You enter a major competition and win through hard work and determination.",
  "sad3":"You enter a major competition and lose, but fairly.",
  "angry3":"You enter a major competition and lose because your adversary cheated.",
  // "happy4":"You find your once-lost favorite shirt and it still fits.",
  // "sad4":"You lose your favorite shirt.",
  // "angry4":"Your favorite shirt is stolen."
}

var scenarios = [
  "Scenario 1: Your best friends are having a reunion...",
  "Scenario 2: Your proposed project idea...",
  "Scenario 3: You enter a major competition...",
]

var ytDict = {
  "happy_pika": "6LXPjbwIDuM",
  "sad_pika": "EKjRVRV7VL8",
  "anger_pika": "Nbs-Dy6RThk",
  "happy_sponge": "mRPrSB17hh8",
  "sad_sponge": "EZ7MXkzrkDc",
  "anger_sponge": "Mv_joub8Vw8",
  "happy_homer": "jvING1JuGl4",
  "sad_homer": "9NGgr3TRRl0",
  "anger_homer": "othu42C6Dgg",
  "happy_elmo": "SN1FN69tkN8",
  "sad_elmo": "r8pSteW2a_Q"
}

const camList = [
  "FaceTime HD Camera",
  "HD USB Camera (05a3:9520)"
]

const armsList = ['No arms','One arm','Two arms']

var defaultPrompt = "Prompt text will appear here.";