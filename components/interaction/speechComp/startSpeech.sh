#!/bin/sh

DEFAULT_PATH="$HOME/etc/speech.conf"

CONFIG_PATH=$DEFAULT_PATH

if test -e $DEFAULT_PATH; then
        CONFIG_PATH=$DEFAULT_PATH
else
        CONFIG_PATH="etc/config"
fi

src/speechComp.py --Ice.Config=$CONFIG_PATH
