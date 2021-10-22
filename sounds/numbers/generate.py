from gtts import gTTS

N = 25
FORMAT = "wav"


def main():
    for i in range(N + 1):
        if i == 0:
            msg = "no points scored"
        else:
            msg = f"{i} points scored"
        path = f"{i}.{FORMAT}"
        gTTS(msg).save(path)
        print(i, msg, path)


if __name__ == "__main__":
    main()
