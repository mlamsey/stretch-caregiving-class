import os
from gtts import gTTS

N = 25
FOLDER = "v1"
FORMAT = "wav"
LANG = "en"
TLD = "co.uk"  # ["co.uk", "com", "ca", "co.in", "ie", "co.za"]


def main():
    msgs, paths = [], []
    for i in range(N + 1):
        if i == 0:
            msgs.append("no points scored")
        else:
            msgs.append(f"{i} points scored")
        paths.append(os.path.join(FOLDER, LANG, TLD, f"{i}.{FORMAT}"))

    msgs.append("nice job!")
    paths.append(os.path.join(FOLDER, LANG, TLD, f"nice-job.{FORMAT}"))

    msgs.append("new high score!")
    paths.append(os.path.join(FOLDER, LANG, TLD, f"new-high-score.{FORMAT}"))

    for msg, path in zip(msgs, paths):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        gTTS(msg, tld=TLD, lang=LANG).save(path)


if __name__ == "__main__":
    main()
