from time import sleep

import vlc


def play_sound(filename):
    # vlc.Instance("--aout=alsa", "--alsa-audio-device=sysdefault:CARD=Headphones")
    player = vlc.MediaPlayer(vlc.Instance("--aout=alsa", "--alsa-audio-device=sysdefault:CARD=CD002"), filename)
    player.play()
    return player.get_length()


time = play_sound("makerobo.mp3") / 1000
if time < 5:
    sleep(5)
else:
    sleep(time)
