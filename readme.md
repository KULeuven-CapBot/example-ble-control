# CapBot control using Bluetooth low energy

This repo exists out of two main parts:
- CapBot firmware that exposes a GATT server (i.e. The [peripheral](./peripheral/))
- A client program that connects to this server (i.e. The [central](./central/))