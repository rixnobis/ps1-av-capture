# ps1-av-capture

A capture board for the PlayStation's analog RGB and line audio. A TI TVP7002
digitises the 15 kHz RGB from the multi-out, a Cypress FX3 carries the pixels to the
host over USB 3 as a UVC camera, and a TI PCM2903C presents the audio as a USB
sound card. This first revision is one channel; the plan is three on one board
behind a hub.

Work in progress: nothing here has been built. Part availability is in
[docs/parts.md](docs/parts.md).

The video front end follows the OSSC (github.com/marqs85/ossc), which uses the
same digitiser on the same signals. The FX3 side follows Cypress application note
AN75779.
