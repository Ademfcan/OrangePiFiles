#! /bin/bash

TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
echo "Starting note localization service at ${TIMESTAMP}" | systemd-cat -p info
python3 /etc/xbot/localizeNoteNt.py | systemd-cat -p info
