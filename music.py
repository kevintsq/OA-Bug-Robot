from time import sleep

import vlc


def play_sound(filename):
    # vlc.Instance("--aout=alsa", "--alsa-audio-device=sysdefault:CARD=Headphones")
    player = vlc.MediaPlayer(vlc.Instance("--aout=alsa", "--alsa-audio-device=sysdefault:CARD=CD002"), filename)
    player.play()
    return player.get_length()


while True:
    time = play_sound("sound.mp3") / 1000
    if time < 6:
        sleep(6)
    else:
        sleep(time)
