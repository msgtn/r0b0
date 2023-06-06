# Rigs

## Configs
The config

```
name: blsm
hostname: localhost
port: 8080
gadgets:
- blossom
- phone
messages:
- msg_func: motion2motor
  tx_gadget: phone
  rx_gadget: blossom
```

The `hostname` and `port` default to `localhost` and `8080` if they are not defined.