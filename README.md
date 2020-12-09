# Mailcam
Arduino software for ESP32-CAM based mail camera. This camera takes a picture of your mail and sends it to your webserver

This Mailcam uses an ESP32-CAM modified for deep sleep and battery power

It deep sleeps until a switch is activated when mail arrives.

When you empty the mailbox another switch sends a message that the box is emptied

The image is taken a few seconds after the switch is activated. A white LED is turned on to act as a weak flash

Note that you need to hack your boards.txt file in Arduino for the software to compile. It is described in the .ino file

You need a webserver that runs PHP to receive the file. The php program is simple and you find it in the initial comments in the .ino file

Secrets are in secrets-mailcam.h

To reprogram the board with OTA simply open the mailbox door so the empty switch is activated. This prevents deep sleep

- mqttTopicAnnounce  "mailcam/announce"   Board announces itself when it wakes up
- mqttTopicDebugSet  "mailcam/debug/set"  Set debug mode
- mqttTopicDebug     "mailcam/debug"      See debug messages here
- mqttTopicReset     "mailcam/reset"      Reset the board
- mqttTopicBattery   "mailcam/battery"    Battery state is reported here
- mqttTopicSnapshot  "mailcam/snapshot"   Set to "on" to take new picture and upload it next time we wakeup to report battery
                                          The topic is reset back to off when picture is taken

