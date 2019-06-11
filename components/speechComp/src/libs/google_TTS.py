#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
from gtts import gTTS
from playsound import playsound

TEMP_FILE = '/tmp/temp.mp3'

def google_tts_say(text):
	tts = gTTS(text, 'es')
	tts.save(TEMP_FILE)
	playsound(TEMP_FILE)
	if os.path.isfile(TEMP_FILE):
		os.remove(TEMP_FILE)  # remove temperory file

def google_tts_busy():
	if os.path.isfile(TEMP_FILE):
		return True
	else:
		return False