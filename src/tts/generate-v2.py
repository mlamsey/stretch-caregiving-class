import os
from gtts import gTTS

# this makes me uncomfy
import urllib3
urllib3.disable_warnings()

FORMAT = "wav"
LANG = "en"
TLD = "co.uk"  # ["co.uk", "com", "ca", "co.in", "ie", "co.za"]


def clean_string(string):
    string = string.lower()
    string.replace(" ", "-")
    string = filter_string_chars(string)
    return string


def filter_string_chars(string):
    naughty_chars = ["/", "!", "?", ".", ",", "@", "#", "$", "%", "^", "&", "*", "(", ")", "_"]
    for c in naughty_chars:
        string = string.replace(c, "")
    return string


class Speak():
    def __init__(self, folder="v2"):
        # get save directory
        folder = clean_string(folder)
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.folder = script_dir + "/" + folder + "/"
        
        # get previously generated words
        rendered_phrases = []
        for (dirpath, dirnames, filenames) in os.walk(self.folder):
            rendered_phrases.extend(filenames)
            break
        self.rendered_phrases = rendered_phrases

    def main(self):
        print("Generating prescribed TTS phrases...")
        self.sws_generate()
    
    def sws_generate(self):
        self.generate_phrase("please hand me the target")
        self.generate_phrase("thank you")
        self.generate_phrase("ready")
        self.generate_phrase("set")
        self.generate_phrase("go")
        self.generate_phrase("thanks for playing!")

    def generate_phrase(self, phrase):
        phrase = filter_string_chars(phrase)
        file_name = clean_string(phrase) + "." + FORMAT
        if file_name not in self.rendered_phrases:
            print("Generating new phrase \"" + phrase + "\"")
            path = self.folder + file_name
            self.generate_tts(phrase, path)

    def generate_tts(self, msg, path):
        try:
            os.makedirs(os.path.dirname(path))
            print("Creating new save path.")
        except OSError:
            pass

        gTTS(msg, tld=TLD, lang=LANG).save(path)

if __name__ == '__main__':
    Speak().main()
