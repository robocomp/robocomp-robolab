#!/bin/sh
LANGUAGE="es-es"
#LANGUAGE="en-en"


#echo "1 SoX Sound .flac with 44100"

sox -t alsa default ./out.flac silence -l 1 0.01 0.5% 1 1.0 0.5%
 
#echo "1 SoX Sound .flac with 16000"
sox out.flac $1.flac rate 16k

#echo "2 Submit to Google Voice Recognition $1.flac"
wget -q -U "Mozilla/5.0" --post-file $1.flac --header="Content-Type: audio/x-flac; rate=16000" -O - "http://www.google.com/speech-api/v1/recognize?lang=${LANGUAGE}&client=chromium" > $1.ret
#echo "3 SED Extract recognized text" 
cat $1.ret > $1.base

cat $1.base | sed 's/.*utterance":"//' | sed 's/","confidence.*//' > $1.txt
cat $1.base | sed 's/.*confidence"://' | sed 's/}.*//' > $1.conf
#echo "4 Remove Temporary Files"
rm $1.flac
rm $1.ret
#echo "5 Show Text "
cat $1.txt $1.conf > $1.combined
cat $1.combined

rm $1.txt
rm $1.conf
