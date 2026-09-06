# Voice control

Both voice nodes record 3-second clips with `arecord` from the USB microphone (`plughw:2,0`, change with `_audio_device:=plughw:1,0`; list devices with `arecord -l`) and transcribe them with Google Speech Recognition, so an internet connection is required.

## Voice navigator (rooms)

```bash
./scripts/run.sh roslaunch companio_voice voice.launch          # mode:=navigator
```

| Say | Action |
|---|---|
| "go to Ward A", "take me to the pharmacy" | `/locations/go_to` (fuzzy name match) |
| "go home" | navigate to *Home* |
| "save this as Ward A" | `/locations/save` at the current pose |
| "contract", "fold", "rest position" | fold into the 30 × 30 × 30 cm rest posture (`/posture/command`) |
| "expand", "unfold", "deploy" | back to the working posture |
| "stop", "cancel" | cancel the goal and stop the motors |
| "forward", "back", "left", "right" | short manual motion (say "stop" to halt) |
| "where are you" | logs the current map pose |
| "over", "exit" | stop listening |

Requires `navigation.launch` (or `mapping.launch` for saving only).

## Voice teleop

```bash
./scripts/run.sh roslaunch companio_voice voice.launch mode:=teleop
```

Commands: forward, left, right, reverse/backward, stop, speed up, speed down, contract, expand, over/exit.

## Voice line following

`rosrun companio_line_follower voice_line_following.py` — see [LINE_FOLLOWING.md](LINE_FOLLOWING.md).
