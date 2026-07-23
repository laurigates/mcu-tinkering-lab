# espdancer RPC Protocol

The wire contract between the firmware (`components/usb_rpc`) and the Python
backend (`facedancer/backends/espdancer.py`). Both sides MUST implement this
exactly. The firmware's `include/usb_rpc.h` mirrors the message IDs here.

## Transport

- **L4**: TCP. The S3 brings up a SoftAP `espdancer-log` (WPA2, pwd
  `espdancer`, subnet `192.168.4.x`). The host connects to `192.168.4.1:4444`.
- **One connection at a time.** A second connect attempt is accepted only
  after the prior socket closes.
- The host may keep the connection open for the device's lifetime; events flow
  device→host on the same socket.

## Frame

Every message (either direction) is one frame:

```
 offset  size  field
   0     1     magic0   = 0xF0
   1     1     magic1   = 0xD0
   2     2     length   = body length, big-endian (includes msg_id + payload)
   4     1     msg_id
   5     N-1   payload  (N = length)
   4+N    1     crc8     over body (msg_id + payload), poly 0x07, init 0x00
```

`length` is the count of bytes from `msg_id` through end-of-payload; i.e. it
is `1 + len(payload)`. Min `length` = 1 (a payload-less message).

Receiver resyncs by scanning for the 2-byte magic; on a bad CRC or absurd
length it drops the frame and keeps scanning.

## CRC8

```c
uint8_t crc = 0x00;
for each byte b in (msg_id + payload):
    crc ^= b;
    for 8 rounds: crc = (crc & 0x80) ? (crc<<1 ^ 0x07) : (crc<<1);
```

## Messages

### Host → device (requests)

| msg_id | name          | payload                                             |
|--------|---------------|-----------------------------------------------------|
| 0x01   | HELLO         | `u16 proto_version` (host's claimed version)        |
| 0x02   | GET_VERSION   | —                                                   |
| 0x10   | CONNECT       | `u8 ep0_max`, `u8 speed` (1=L,2=F,3=H), `u8 quirks` |
| 0x11   | DISCONNECT    | —                                                   |
| 0x12   | CONFIGURE_EP  | `u8 addr`, `u8 dir`(0=OUT,1=IN), `u8 type`, `u16 max_pkt` |
| 0x13   | SEND          | `u8 ep_num`, `u16 data_len`, `u8 data[data_len]`    |
| 0x14   | STALL         | `u8 ep_num`, `u8 dir`                               |
| 0x15   | CLEAR_HALT    | `u8 ep_num`, `u8 dir`                               |
| 0x16   | ACK_STATUS    | `u8 dir`, `u8 ep_num`                               |
| 0x17   | SET_ADDRESS   | `u8 addr`, `u8 defer`                               |
| 0x18   | RESET         | —                                                   |

`quirks` bit 0 = `manual_set_address` (mirror moondancer `QuirkFlag`).

### Device → host (replies + events)

| msg_id | name          | payload                                              |
|--------|---------------|------------------------------------------------------|
| 0x81   | HELLO_REPLY   | `u16 proto_version` (`USB_RPC_PROTO_VERSION`), `char[16] fw_version` |
| 0x82   | VERSION_REPLY | `char[]`                                             |
| 0x90   | OK            | `u8 in_reply_to` (the request msg_id)                |
| 0x91   | ERROR         | `u8 in_reply_to`, `u8 code`, `char[] msg`            |
| 0xA0   | EVENT         | `u8 event_type`, event-specific payload (below)      |

EVENT payload types:

| event_type | name           | extra payload                              |
|------------|----------------|-------------------------------------------|
| 0x01       | BUS_RESET      | —                                         |
| 0x02       | SETUP          | `u8 ep_num`, `u8 setup[8]`                 |
| 0x03       | OUT_PACKET     | `u8 ep_num`, `u16 data_len`, `u8 data[]`  |
| 0x04       | SEND_COMPLETE  | `u8 ep_num`                               |
| 0x05       | NAK            | `u8 ep_num`                               |

These map 1:1 onto facedancer's `InterruptEvent` (codes 10–13 in
`moondancer.py`) and `HydradancerEvent` (0–3 in `hydradancer.py`):
`SETUP`→`USB_RECEIVE_CONTROL`/`create_request`, `OUT_PACKET`→`handle_data_available`,
`NAK`→`handle_nak`, `BUS_RESET`→bus reset.

### Error codes

| code | meaning            |
|------|--------------------|
| 0x01 | NOT_IMPLEMENTED    |
| 0x02 | BAD_FRAME          |
| 0x03 | BAD_ARG            |
| 0x04 | USB_FAILED         |

## Handshake (Milestone 0 gate)

1. Host opens TCP to `192.168.4.1:4444`.
2. Host sends `HELLO` with `proto_version = 0x0001`.
3. Device replies `HELLO_REPLY` with its `proto_version` + `fw_version` string.
4. Host sends `GET_VERSION`; device replies `VERSION_REPLY`.
5. Any other request before `CONNECT` returns `ERROR NOT_IMPLEMENTED`
   (Milestone 0) / is honored (Milestone 1+).

A minimal handshake from the shell:

```sh
# Connect to 'espdancer-log' WiFi first.
# HELLO: magic(2) + len_be(2, =3) + msg_id(0x01) + proto(0x0001) + crc8
printf '\xf0\xd0\x00\x03\x01\x00\x01' | nc 192.168.4.1 4444 | xxd
```

## Versioning

`USB_RPC_PROTO_VERSION` starts at `0x0001`. Incompatible changes bump the
major (high) byte; backwards-compatible additions bump the low byte. The host
advertises its version in HELLO; the device logs a warning on mismatch but
keeps the higher of the two it can speak.