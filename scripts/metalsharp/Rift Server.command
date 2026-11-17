#!/bin/zsh
# Starts MetalVR's Rift server in Terminal, so the Screen Recording permission that lets the headset show a game's
# window belongs to Terminal. Ctrl-C stops it.
exec $HOME/.metalsharp/metalvr-runtime/bin/metalvr rift --verbose 2>&1 | tee $HOME/.metalsharp/logs/rift-server.log
