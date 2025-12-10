#!/bin/bash

ps -a -o pid= | while read pid; do
    kill -9 "$pid" 2>/dev/null
done