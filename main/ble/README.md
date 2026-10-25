# Documentation of the BLE interface for LispBM

## Introduction

The LispBM BLE interface lets VESC Express expose custom services, communicate
with BLE peripherals, or perform both roles at the same time. The built-in VESC
Tool service can remain available alongside scripted services and outgoing
connections.

Use the **server** API to expose values to a phone or computer, or to make the
board act as a HID device. Use the **client** API to scan for peripherals,
connect, discover their services, exchange data, and handle pairing.

## Contents

- [Configuration](#configuration)
- [Quick start: custom server](#quick-start-custom-server)
- [Data formats and limits](#data-formats-and-limits)
- [Server API](#server-api)
- [Server write events](#server-write-events)
- [Advertising](#advertising)
- [Client API](#client-api)
- [Client events](#client-events)
- [Pairing outgoing connections](#pairing-outgoing-connections)
- [Multiple peripherals](#multiple-peripherals)
- [HID](#hid)
- [Troubleshooting](#troubleshooting)
- [Implementation and references](#implementation-and-references)

## Configuration

In VESC Tool, open **VESC Express > Bluetooth**. Configure the VESC service and
LispBM roles independently, write the configuration, and reboot. Changes to
these settings take effect after reboot.

| `ble-mode` | Built-in VESC Tool service |
| --- | --- |
| 0 | Disabled |
| 1 | Enabled without mandatory encryption |
| 2 | Enabled with encryption and authenticated PIN pairing |

| `ble-mode-lbm` | LispBM role | Server API | Client API |
| --- | --- | --- | --- |
| 0 | Disabled | No | No |
| 1 | Server | Yes | No |
| 2 | Client | No | Yes |
| 3 | Client and server | Yes | Yes |

Only the extensions for the selected roles are enabled. Check the saved role
with `(conf-get 'ble-mode-lbm)`. BLE is disabled when both modes are 0.

The firmware supports one incoming connection shared by the VESC and scripted
services, plus up to four outgoing client connections. The services on an
incoming connection share its encryption state. Encrypted VESC mode requires
both encryption and authentication to access the VESC transport. Generic
scripted characteristics do not automatically inherit those access requirements.
Outgoing pairing has its own policy, configured through the client API.

GATT organizes data into services, characteristic values, and descriptors. A
characteristic holds application data, while its descriptors provide metadata
or control features such as notification subscriptions.

For the server role, set these capacities before rebooting:

| Setting | What it counts |
| --- | --- |
| **BLE Service Capacity** | Maximum number of scripted services, including a scripted HID service |
| **BLE Characteristic and Descriptor Capacity** | Total number of explicitly defined characteristic values and descriptors across those services |

A characteristic counts as one entry. Each explicit descriptor counts as another.
Automatic CCCDs are managed by NimBLE and do not consume an explicit descriptor
entry unless the script includes one in its definition. The built-in GAP, GATT,
and VESC services do not consume these scripted capacities.

For example, one service containing two characteristics and one explicit
descriptor needs a service capacity of 1 and an attribute capacity of 3.

With `ble-mode = 0` and `ble-mode-lbm = 2`, the board acts only as a client. It
does not advertise or register local server services. Use USB or CAN for
configuration. This mode avoids the VESC transport buffers and scripted server
allocations, but the shared BLE host and controller still use memory.

## Quick start: custom server

Additional standalone scripts are available in
[lbm_examples/ble_scripting](../../lbm_examples/ble_scripting/), including
scanning, connection and discovery, raw advertising packets, explicit CCCDs,
and incoming write events.

Enable the server role, set service capacity to at least 1 and attribute capacity
to at least 1, then save and reboot. This example exposes one readable,
writable characteristic with notifications:

```clj
(ble-set-name "device-name")
(ble-start-app)

; Remove services left by a previous run of this script.
(map ble-remove-service (reverse (ble-get-services)))

(def service-uuid [
    0xad 0x2d 0x03 0xdb 0x53 0x35 0x43 0xaf
    0x9a 0x70 0xea 0x54 0x63 0xad 0x38 0x13
])
(def value-uuid [
    0xae 0x2d 0x03 0xdb 0x53 0x35 0x43 0xaf
    0x9a 0x70 0xea 0x54 0x63 0xad 0x38 0x13
])

(def handles (ble-add-service service-uuid `(
    (
        (uuid . ,value-uuid)
        (prop . (prop-read prop-write prop-notify))
        (max-len . 20)
        (default-value . [42])
    )
)))
(def value-handle (ix handles 1))

(ble-attr-set-value value-handle [43])
(ble-attr-get-value value-handle)
```

Connect with a BLE client, discover the service, and read or write the
characteristic. To receive subsequent updates, enable notifications on the
client before calling `ble-attr-set-value` again. The firmware creates the
required Client Characteristic Configuration Descriptor (CCCD) automatically.

The UUIDs above are example custom UUIDs. The service is available through GATT
discovery even though default advertising does not list its UUID. To advertise
it explicitly, use the [custom advertising example](#custom-advertising-example).

The cleanup line removes **all** scripted services. Omit it if another script
owns services that must remain registered. Reloading Lisp does not reset the BLE
server. On later runs, `ble-start-app` returns `nil`, and `ble-set-name` cannot
change the name until reboot.

## Data formats and limits

### UUIDs and handles

Server definitions and client discovery events use UUID byte arrays in display
order, also called big endian. For example, UUID `0x1812` is `[0x18 0x12]`.
Accepted UUID lengths are 2, 4, or 16 bytes for 16-, 32-, or 128-bit UUIDs.
A 32-bit UUID discovered over ATT appears in its expanded 128-bit Bluetooth Base
UUID form.

Advertising arrays contain raw wire bytes. Multi-byte fields such as service
UUIDs and appearance values use the byte order specified by Bluetooth, typically
little endian. For example, the HID service is `[0x12 0x18]` in advertising.

A **service handle** identifies a service. An **attribute handle** identifies a
characteristic value or descriptor. Do not pass a service handle to
`ble-attr-get-value` or `ble-attr-set-value`. Always use returned or discovered
handles instead of hard-coding numeric values. Remote handles belong to a
particular peer and connection session.

### Byte arrays and strings

Attribute values and advertising field payloads are byte arrays. LispBM strings
include a terminating zero byte, which is also transmitted if you use a string
as data. To send `hello` without that byte:

```clj
(ble-attr-set-value value-handle (buf-resize "hello" -1 'copy))
```

Use an ordinary null-terminated string for `ble-set-name`.

### Payload sizes

`MTU` below means the negotiated ATT MTU for the connection. The firmware sets
its preferred MTU to 256, but the negotiated value also depends on the peer.
At MTU 23, one ordinary read carries at most 22 value bytes, and one write or
notification carries at most 20. At MTU 256, those limits are 255 and 253 bytes.

| Operation | Limit in the current implementation |
| --- | --- |
| Advertising or scan response | 31 bytes per packet, including field headers |
| Scripted device name | 30 bytes, excluding the terminating zero |
| Default advertised name | Up to 29 name bytes in the scan response |
| Ordinary client read | One ATT response, at most `MTU - 1` value bytes, within the 512-byte event buffer |
| Long client read | Up to 512 bytes, assembled from multiple ATT responses |
| Client write | At most `min(255, MTU - 3)` bytes, without long-write support |
| Received client notification or indication | One packet, at most `MTU - 3` value bytes, within the 512-byte event buffer |
| HID report map | 1 to 512 bytes |
| HID report payload | 1 to 512 declared bytes, with input notifications also limited to `MTU - 3` |

The 255-byte client write limit is an API restriction. The 512-byte limit is the
maximum BLE attribute value size and the size of the current client event data
buffer. A larger buffer does not increase the negotiated packet size.

## Server API

Server functions require `ble-mode-lbm` 1 or 3. Invalid argument types generally
raise `type_error`. Invalid state, handles, capacity, or stack failures generally
raise `eval_error`. Allocation failures can raise `out_of_memory`.

### `ble-start-app`

```clj
(ble-start-app)
```

Enable scripted services on the shared BLE host. Call this before adding
services. It returns `t` on the first successful call and `nil` if scripted
services have already started. Initialization or host startup failures raise
`eval_error`.

The existing VESC service and incoming connection remain available. There is no
script API to stop the shared BLE host. Reboot to reset its state.

### `ble-set-name`

```clj
(ble-set-name "device-name")
```

Set the runtime GAP Device Name and the name used by default scripted
advertising. Call this **before the first `ble-start-app`**. It returns `t` when
accepted and `nil` after scripted services have started. A name longer than 30
bytes raises `eval_error`.

Default scan responses contain at most 29 name bytes. A 30-byte name is marked
as shortened, while the full name remains available through the GAP Device Name
characteristic. Custom advertising uses the name bytes supplied by the script.

This runtime name does not update the saved `ble-name` setting, whose storage
holds eight name bytes and a terminating zero.

### `ble-add-service`

```clj
(ble-add-service service-uuid characteristics)
```

Add a primary service after `ble-start-app`. `characteristics` is a list of
association lists, each describing one characteristic:

| Key | Required | Value |
| --- | --- | --- |
| `uuid` | Yes | UUID byte array of length 2, 4, or 16 |
| `prop` | Yes | List of characteristic property symbols |
| `max-len` | Yes | Positive maximum value length in bytes |
| `default-value` | No | Nonempty byte array no longer than `max-len`, default `[0]` |
| `descr` | No | List of descriptor association lists |

Available characteristic properties:

| Symbol | Client operation |
| --- | --- |
| `prop-read` | Read the value |
| `prop-write` | Write with an ATT response |
| `prop-write-nr` | Write without an ATT response |
| `prop-notify` | Subscribe to notifications |
| `prop-indicate` | Subscribe to indications |

Descriptor definitions accept `uuid`, `max-len`, and optional `default-value`.
Generic descriptors are readable and writable. The generic API does not expose
per-attribute encryption or authentication flags. Use maximum lengths within
the BLE attribute limit of 512 bytes.

Service UUIDs must be unique. GAP (`0x1800`), GATT (`0x1801`), and the built-in
VESC transport UUID are reserved, even if the VESC transport is disabled.

The return value is a list containing the service handle, then each
characteristic value handle followed by its explicitly defined descriptor
handles, in definition order. For example:

```text
(service characteristic-1 descriptor-1 characteristic-2 ...)
```

#### Notification subscriptions and CCCDs

For `prop-notify` or `prop-indicate`, NimBLE automatically creates a CCCD with
UUID `0x2902`. The client writes this descriptor to enable updates. A CCCD
created automatically is omitted from the returned script handle list.

For compatibility, a script may explicitly include this descriptor:

```clj
(descr . (
    (
        (uuid . [0x29 0x02])
        (max-len . 2)
        (default-value . [0 0])
    )
))
```

Its returned handle refers to NimBLE's CCCD, rather than a second descriptor.
The explicit entry counts toward scripted attribute capacity. The script can
read its cached subscription value but cannot write it with
`ble-attr-set-value`. Subscriptions are cleared when the incoming peer
disconnects.

### `ble-remove-service`

```clj
(ble-remove-service service-handle)
```

Remove a scripted service and its characteristics and descriptors. Returns `t`
on success. Services must be removed in reverse creation order. Removing an
unknown service or a service that is not the most recently added raises
`eval_error`.

To remove all scripted services:

```clj
(map ble-remove-service (reverse (ble-get-services)))
```

### `ble-attr-get-value`

```clj
(ble-attr-get-value attribute-handle)
```

Return the current local characteristic or descriptor value as a byte array.
An unknown attribute handle raises `eval_error`.

### `ble-attr-set-value`

```clj
(ble-attr-set-value attribute-handle data)
```

Copy `data` into a local characteristic or descriptor and return `t` on success.
The value must fit its declared maximum length. HID report values must also
have exactly their declared length. Unknown handles, writes to CCCDs, invalid
lengths, or notification failures raise `eval_error`.

For a characteristic with an active subscription, setting the value also
attempts to send a notification or indication. Without a subscription, only
the local value is updated. The value is stored before transmission is
attempted, so a transmission error does not undo the local update.

This function does not split large values across notifications. Keep transmitted
values within `MTU - 3` bytes.

### `ble-get-services`

```clj
(ble-get-services)
```

Return active scripted service handles in creation order. Returns `nil` if none
exist, including before `ble-start-app`. Built-in services are excluded.

### `ble-get-attrs`

```clj
(ble-get-attrs service-handle)
```

Return the characteristic and explicit descriptor handles for a scripted service
in definition order. This is the list returned by `ble-add-service`, without
the first service-handle element. An invalid service handle raises `eval_error`.

## Server write events

Enable `event-ble-rx` to receive writes made by the incoming client. These use
the normal LispBM event mechanism, independently of the polled client queue.
Each event has this layout:

```clj
(event-ble-rx attribute-handle data-array)
```

The value has already been updated when the event is delivered. An explicit
CCCD also produces this event when the client changes its subscription. Local
calls to `ble-attr-set-value` do not generate a remote-write event.

```clj
(defun ble-event-handler ()
    (loopwhile t
        (recv
            ((event-ble-rx (? handle) (? data))
                (print (list 'ble-write handle data)))
            (_ nil)
        )
    )
)

(event-register-handler (spawn ble-event-handler))
(event-enable 'event-ble-rx)
```

An application with an existing event handler should add the BLE pattern to that
handler instead of registering a second one.

## Advertising

These functions belong to the server role. Default advertising is usually
sufficient. Use custom packets when a client must find a particular service UUID
or needs additional discovery data.

### Default packets

After `ble-start-app`, default advertising contains:

- Flags `0x06`, indicating general discoverability and no BR/EDR support.
- The VESC transport service UUID when `ble-mode` is nonzero.
- The HID service UUID and current appearance when a scripted HID service exists.

The default scan response contains the device name. Other scripted service
UUIDs are not added automatically. Advertising stops while the incoming peer is
connected and resumes after it disconnects.

### `ble-conf-adv`

```clj
(ble-conf-adv nil)
(ble-conf-adv t advertising-data scan-response-data)
```

Select default packets with `nil`, or custom packets with `t`. Each custom
packet argument accepts a raw byte array, a nonempty list of advertising fields,
or `nil` to retain the previous custom packet. Custom packets initially contain
no data. Pass `[]` to explicitly clear a packet.

Returns `t` on success or `nil` for an internal BLE update failure. Invalid packet
types raise `type_error`. Packets longer than 31 bytes raise `eval_error`.

Call this before or after `ble-start-app`. After startup, it restarts advertising
when no incoming peer is connected. Custom packets replace the defaults
entirely, so include any required VESC or HID discovery fields yourself.

A field list uses `(type . payload-array)` entries. For example:

```clj
(def advertising-data '((flags . [0x06])))
(def scan-response-data '((name-complete . [0x74 0x65 0x73 0x74])))
(ble-conf-adv t advertising-data scan-response-data)
```

Equivalent raw packets:

```clj
(ble-conf-adv t [0x02 0x01 0x06] [0x05 0x09 0x74 0x65 0x73 0x74])
```

Both advertise the name `test`. Set the GAP name separately with `ble-set-name`
if it should match.

### Advertising fields

Each field contains a length byte, a type byte, and its payload. The length
byte counts the type and payload, but not itself. Each field therefore uses
`2 + payload-length` bytes. The advertising and scan response packets each have
their own 31-byte limit. Scan response data is available to active scanners.

| Symbol | Type | Payload |
| --- | --- | --- |
| `flags` | `0x01` | Discovery and transport flags |
| `incomplete-uuid-16` | `0x02` | Incomplete list of 16-bit service UUIDs |
| `complete-uuid-16` | `0x03` | Complete list of 16-bit service UUIDs |
| `incomplete-uuid-32` | `0x04` | Incomplete list of 32-bit service UUIDs |
| `complete-uuid-32` | `0x05` | Complete list of 32-bit service UUIDs |
| `incomplete-uuid-128` | `0x06` | Incomplete list of 128-bit service UUIDs |
| `complete-uuid-128` | `0x07` | Complete list of 128-bit service UUIDs |
| `name-short` | `0x08` | Shortened device name |
| `name-complete` | `0x09` | Complete device name |
| `tx-power-level` | `0x0a` | Transmit power in dBm, as a signed byte |
| `device-id` | `0x10` | Device ID profile data |
| `conn-interval-range` | `0x12` | Minimum and maximum peripheral connection intervals |
| `service-data-16` | `0x16` | 16-bit service UUID followed by service data |
| `appearance` | `0x19` | 16-bit appearance value |
| `service-data-32` | `0x20` | 32-bit service UUID followed by service data |
| `service-data-128` | `0x21` | 128-bit service UUID followed by service data |
| `manufacturer-data` | `0xff` | Company identifier followed by manufacturer data |

Use an integer type from 1 to 255 for fields without a named symbol. Payload
arrays are copied as supplied. The field-list API adds the length and type bytes
but does not convert payload byte order or remove string terminators.

### `ble-conf-adv-set-channels`

```clj
(ble-conf-adv-set-channels '(37 38 39))
```

Select a nonempty list of advertising channels from 37, 38, and 39. All three are
selected by default. Returns `t`. Invalid types raise `type_error`, and an empty
list or invalid channel raises `eval_error`.

### `ble-conf-adv-set-interval`

```clj
(ble-conf-adv-set-interval minimum maximum)
```

Set advertising intervals in units of 0.625 ms. Both values must be between
`0x0020` and `0x4000` inclusive, corresponding to 20 ms through 10.24 s, and
`minimum` must not exceed `maximum`. Returns `t` or raises `eval_error` for an
invalid interval. Invalid types raise `type_error`.

The default scripted intervals are `0x20` to `0x40`, or 20 to 40 ms. For example,
`(ble-conf-adv-set-interval 0x400 0x800)` selects 640 to 1280 ms.

Channel and interval setters update the parameters for the **next advertising
start**. They do not restart active advertising. Configure them before
`ble-start-app`, or reapply `ble-conf-adv` afterward to restart advertising with
the new parameters when disconnected.

### Custom advertising example

This example advertises the service from the quick start. Configure it after
defining the service, or before `ble-start-app` if preparing all settings first.
The service UUID bytes are reversed relative to the server definition:

```clj
(def advertising-data '(
    (flags . [0x06])
    (incomplete-uuid-128 . [
        0x13 0x38 0xad 0x63 0x54 0xea 0x70 0x9a
        0xaf 0x43 0x35 0x53 0xdb 0x03 0x2d 0xad
    ])
))
(def scan-response-data `(
    (name-complete . ,(buf-resize "device-name" -1 'copy))
))
(ble-conf-adv t advertising-data scan-response-data)
```

The advertising packet uses 21 bytes and the scan response uses 13. This is a
custom discovery configuration and does not reproduce the default packets.
The incomplete UUID list allows other 128-bit services, such as the VESC
transport, to remain available without listing them in this packet.

## Client API

Client functions require `ble-mode-lbm` 2 or 3. They do not require
`ble-start-app` or scripted server capacity.

### Requests and completion

Client requests return `t` when accepted, or a nonzero integer NimBLE error code
when rejected. Invalid arguments raise `type_error`. **Compare the result with
`t` explicitly**, because a nonzero error code is also truthy in LispBM.

Acceptance does not mean the remote operation has completed. Consume events to
obtain its result. Only one GATT procedure or requested security procedure can
run on a connection at a time. Wait for its completion before starting the next
one. Different connections can have procedures in progress simultaneously.

A typical client sequence is:

1. Scan and select the address and address type from `ble-scan-result`.
2. Connect and wait for a successful `ble-connect-result`.
3. If required, pair and wait for `ble-security-result`.
4. Exchange MTU and wait for `ble-mtu-result`.
5. Discover services, characteristics, and descriptors, waiting for each
   `ble-procedure-done` before starting the next discovery procedure.
6. Read, write, or subscribe using the discovered handles.
7. Process notifications and connection events in one event dispatcher.

Connection establishment is sequential. Wait for one connection attempt to
finish before starting another.

### `ble-client-scan`

```clj
(ble-client-scan duration-ms)
```

Start an active scan with duplicate filtering. Duration must be an integer from
1 to 60000 milliseconds. `(ble-client-scan 0)` requests cancellation. A normal
scan completion produces `ble-scan-done`. The cancellation path does not
synthesize a completion event, so do not wait for one to acknowledge cancellation.

Scan results contain an address, address type, RSSI, and raw advertising payload.
The payload must be decoded by the script. Starting a connection cancels an
active scan.

### `ble-client-connect`

```clj
(ble-client-connect address address-type)
(ble-client-connect address address-type timeout-ms)
(ble-client-connect address address-type timeout-ms io-cap)
(ble-client-connect address address-type timeout-ms io-cap bond)
```

Connect to a six-byte address in displayed MAC order. For example,
`AA:BB:CC:DD:EE:FF` is `[0xaa 0xbb 0xcc 0xdd 0xee 0xff]`. Use address type 0 for
public or 1 for random, preserving the type reported by the scan.

The default timeout is 10000 ms. Explicit timeouts must be integers from 1 to
60000 ms. Supplying `io-cap` also requests pairing immediately after connection.
`bond` defaults to `t`. See [pairing](#pairing-outgoing-connections) for I/O modes.

The same function provides connection management:

| Call | Result or action |
| --- | --- |
| `(ble-client-connect)` | First active outgoing connection handle, or `nil` |
| `(ble-client-connect t)` | List of all active outgoing connection handles |
| `(ble-client-connect connection)` | Current RSSI in dBm for that outgoing connection, or `nil` on failure |
| `(ble-client-connect nil)` | Request disconnection of all outgoing peers and cancellation of a pending connection attempt |
| `(ble-client-connect nil connection)` | Request disconnection of that outgoing peer |
| `(ble-client-connect 'limit)` | Current logical outgoing connection limit |
| `(ble-client-connect 'limit count)` | Set that limit, subject to compiled capacity and active connections |

Connection handle **0 is valid**. The value 65535 denotes no connection. Client
disconnect requests do not disconnect the incoming VESC or scripted server peer.
Wait for connection or disconnection events before assuming the state changed.

### Selecting a connection

GATT operations accept an optional connection handle as their last argument.
Omitting it selects the first active outgoing connection. Specify it explicitly
when more than one peer may be connected.

Square brackets in the following signatures indicate optional arguments. They
are documentation notation, not LispBM array syntax.

### `ble-client-exchange-mtu`

```text
(ble-client-exchange-mtu [connection])
```

Request ATT MTU exchange using the local preferred MTU of 256. The result is
reported by `ble-mtu-result`. Use the negotiated value to calculate packet
limits. An exchange does not guarantee an MTU of 256.

### Discovery

```text
(ble-client-discover-services [connection])
(ble-client-discover-chars start-handle end-handle [connection])
(ble-client-discover-descriptors value-handle end-handle [connection])
```

Service discovery returns all primary services. Characteristic discovery uses
an inclusive range, normally the start and end handles reported for a service.
Descriptor discovery starts after the characteristic value handle. Its end must
be the handle before the next characteristic declaration, or the service end
for the last characteristic.

Each procedure emits zero or more discovery records followed by
`ble-procedure-done`. Its operation code is 4 for services, 5 for characteristics,
and 6 for descriptors. Completion status 0 means discovery succeeded.

Characteristic records contain both the value handle and declaration handle.
Use the **value handle** for reading and writing. Discover the CCCD for
subscriptions instead of assuming a numeric offset.

### `ble-client-read` and `ble-client-read-long`

```text
(ble-client-read attribute-handle [connection])
(ble-client-read-long attribute-handle [connection])
```

`ble-client-read` performs one ordinary ATT read and returns at most `MTU - 1`
value bytes. It does not automatically read the remainder of a longer value.

`ble-client-read-long` reads from offset zero and assembles up to 512 bytes
through the ATT long-read procedure. Use it for values such as HID report maps.
It emits one final `ble-read-result`, containing the complete assembled value on
success or an empty array on error.

Both operations report completion through `ble-read-result`. Check its status
before using the value.

### `ble-client-write`

```text
(ble-client-write attribute-handle data [response [connection]])
```

Write a byte array. `response` is 1 by default for an acknowledged ATT write,
or 0 for a write without response. To select a connection explicitly, supply
the response argument as well:

```clj
(ble-client-write value-handle [1 2 3] 1 connection)
```

The payload must fit `min(255, MTU - 3)` bytes. Larger arrays are rejected by
argument validation or the host. Long writes using prepare/execute are not
implemented.

An acknowledged write completes with `ble-write-result`. A write without
response returns after queuing and does **not** produce `ble-write-result`.
Its acceptance does not confirm receipt by the peer.

### `ble-client-subscribe`

```text
(ble-client-subscribe cccd-handle mode [connection])
```

Write a discovered CCCD using an acknowledged ATT write. Mode 0 disables
updates, 1 enables notifications, and 2 enables indications. Completion uses
`ble-write-result`. Incoming updates use `ble-notification`.

Pass the descriptor handle with UUID `0x2902`, not the characteristic value
handle. The remote characteristic must support the selected update type.

### `ble-client-event`

```clj
(ble-client-event)
(ble-client-event t)
```

Return and consume one queued client event, or `nil` when the queue is empty.
The `t` form inserts a connection field after the event symbol, allowing one
dispatcher to route events from multiple peers. If Lisp memory allocation fails,
the event remains queued for a later retry.

Exactly one Lisp context should consume this queue. Include a sleep between
polls so other Lisp work and the GUI can run. This diagnostic loop prints all
events and is itself the queue consumer:

```clj
(loopwhile t {
    (var event (ble-client-event t))
    (if event (print event))
    (sleep 0.01)
})
```

Stop this loop before starting an application dispatcher. Polling client events
does not require `event-enable`.

### `ble-client-dropped`

```clj
(ble-client-dropped)
```

Return the cumulative number of client events lost since boot because the queue
could not accept them. Treat an increase as a loss of application state. Stop
scanning or disconnect the affected peers, drain queued events, and repeat
connection setup and discovery before trusting results.

The queue is allocated on the first scan or connection request. It is bounded
and reserves space for completion and connection events, but that reservation
does not guarantee delivery if the consumer stops keeping up. The queue and
connections survive a Lisp reload. Stop old consumers and discard stale events
before beginning a new session. Reboot resets client state and the drop counter.

## Client events

The default `(ble-client-event)` layouts are:

```text
(ble-scan-result address-array address-type rssi advertising-array)
(ble-scan-done reason)
(ble-connect-result connection-handle status)
(ble-disconnected connection-handle reason)
(ble-service start-handle end-handle uuid-array)
(ble-characteristic value-handle declaration-handle properties uuid-array)
(ble-descriptor handle uuid-array)
(ble-procedure-done operation status)
(ble-read-result handle status value-array)
(ble-write-result handle status)
(ble-notification handle status value-array indication)
(ble-mtu-result negotiated-mtu status)
(ble-passkey-action connection-handle action number)
(ble-security-result connection-handle status encrypted authenticated bonded)
```

For results with a `status`, 0 means success. Disconnect and scan completion
reasons are native NimBLE codes, not ordinary success flags. A failed connection
reports connection handle 65535. Do not use data from failed read or notification
events.

`properties` is the standard characteristic bitmask:

| Bit | Meaning |
| --- | --- |
| `0x02` | Read |
| `0x04` | Write without response |
| `0x08` | Write with response |
| `0x10` | Notify |
| `0x20` | Indicate |

`indication` is 0 for a notification or 1 for an indication. The security fields
`encrypted`, `authenticated`, and `bonded` are also numeric 0/1 flags.

With `(ble-client-event t)`, **every** event has an extra connection field
immediately after its symbol. For example:

```text
(ble-read-result connection attribute-handle status value-array)
(ble-notification connection attribute-handle status value-array indication)
(ble-procedure-done connection operation status)
(ble-connect-result connection connection-handle status)
(ble-disconnected connection connection-handle reason)
(ble-passkey-action connection connection-handle action number)
(ble-security-result connection connection-handle status encrypted authenticated bonded)
```

The existing connection-handle fields are retained, so some events contain the
handle twice. Scan events use 65535 in the added connection field because they
are not connection-specific.

## Pairing outgoing connections

### `ble-client-security`

```text
(ble-client-security connection [io-cap [bond]])
(ble-client-security connection 'forget)
```

Start pairing or restore encryption using a stored bond. The connection argument
is required. The default I/O capability is 4, and bonding defaults to `t`.

| I/O capability | Script interaction | Requested protection |
| --- | --- | --- |
| 0 | Display a PIN for entry on the peer | Encryption with MITM authentication |
| 1 | Display a PIN or confirm matching numbers | Encryption with MITM authentication |
| 2 | Enter the PIN shown by the peer | Encryption with MITM authentication |
| 3 | No input or display | Encryption without mandatory MITM authentication |
| 4 | Keyboard and display, including numeric confirmation | Encryption with MITM authentication |

The peer's capabilities and pairing mode determine the actual interaction.
Numeric comparison requires Secure Connections. Passkeys can also be used with
Legacy Pairing.

`bond = nil` avoids requesting storage of new keys. It does not delete an
existing bond. The supplied BLE build profiles enable NimBLE NVS persistence,
so stored bonds can survive reboot. Reconnect and request security to restore
encryption from the bond.

Wait for `ble-security-result` before starting GATT work. For a requested
security procedure, status 0 requires encryption and, for I/O modes other than
3, authentication. Unsuitable existing keys cause an error instead of an
automatic retry with weaker protection.

If a peer requests security immediately on connection, select the policy in the
connect call. For example, this requests PIN input and bonding:

```clj
(ble-client-connect address address-type 10000 2 t)
```

Otherwise, wait for `ble-connect-result`, then call `ble-client-security`.
Outgoing security settings do not replace the incoming server's configured PIN
policy.

### `ble-client-passkey`

```clj
(ble-client-passkey connection response)
```

Respond to a pending `ble-passkey-action`. The response depends on the action:

| Action | Required response |
| --- | --- |
| 2: input | Read the six-digit PIN from the peer and submit it as an integer |
| 3: display | Choose a six-digit PIN, show it to the operator, and submit the same integer for entry on the peer |
| 4: numeric comparison | Show the event's `number` with leading zeros, then submit `t` to accept or `nil` to reject after comparison |

For example:

```clj
(ble-client-passkey connection 123456)
```

PIN values range from 0 to 999999. The integer `123` represents `000123`.
Only numeric comparison uses the `number` carried by the event as the displayed
comparison value. A PIN response can arrive later from a GUI or REPL.

The pending action is tracked per connection. Wrong response types, invalid
PINs, and responses without a matching pending action are rejected. Disconnect
the peer to cancel PIN entry. Out-of-band pairing actions are not implemented.
Numeric comparison requires an operator's decision and must not be accepted
automatically.

### Removing a bond

```clj
(ble-client-security connection 'forget)
```

Delete the connected peer's stored keys and disconnect it. Wait for
`ble-disconnected`, then reconnect and pair again. This is useful when a peer
has lost its own keys. The operation is rejected while a GATT or security
procedure is active.

Unsolicited repeat pairing does not automatically erase an outgoing peer's
bond. Keys belong to the peer identity, not to a particular service. Entering a
PIN does not save that PIN as an application configuration setting.

## Multiple peripherals

The current API supports at most four outgoing peers. The initial logical limit
is the smaller of four and the compiled connection capacity remaining after
reserving one incoming slot when either the VESC or scripted server is enabled.
The supplied BLE profiles configure five NimBLE connection slots.

```clj
(ble-client-connect 'limit)
(ble-client-connect 'limit 2)
(ble-client-connect t)
```

The logical limit must be at least 1 and cannot exceed the available compiled
capacity. Lowering it below the number of active plus pending connections fails
with a busy error. It resets on reboot, so set it in the startup script if
needed. Lowering it does not release host or controller memory.

Use explicit connection arguments and `(ble-client-event t)` for all peers.
One dispatcher must own the shared client event queue. Store discovered handles,
MTU, security state, and pending operations separately for each connection.
Discard a peer's discovered handles when it disconnects.

For firmware builds, C3/S3 profiles also configure
`CONFIG_BT_CTRL_BLE_MAX_ACT=7` to allow connections, scanning, and advertising.
Changing these compiled pools can reduce RAM use. The logical Lisp connection
limit alone does not resize them.

## HID

HID has two distinct uses:

- **Board as a HID device:** enable the server role and expose a keyboard, mouse,
  gamepad, or other HID interface to a computer or phone.
- **Board as a HID host:** enable the client role and connect to an external BLE
  keyboard, mouse, or controller.

### `ble-hid-add-service`

```text
(ble-hid-add-service report-map reports [appearance])
```

Call `ble-start-app` first. `report-map` is a byte array containing a HID Report
Descriptor, with a length of 1 to 512 bytes. `reports` is a list of 1 to 16
`(id type size)` entries:

| Field | Meaning |
| --- | --- |
| `id` | Report ID from 0 to 255. Use 0 when the descriptor has no Report ID items |
| `type` | 1 for input, 2 for output, or 3 for feature |
| `size` | Fixed report payload length from 1 to 512 bytes, excluding the report ID |

IDs, types, and sizes must match the report map. The firmware checks the entry
ranges and uniqueness but does not parse the descriptor to verify that match.
Each `(id, type)` pair must be unique. Do not mix unnumbered reports with
numbered reports in one service.

The optional 16-bit appearance defaults to `0x03c0`, generic HID. Common values
are `0x03c1` for a keyboard, `0x03c2` for a mouse, and `0x03c4` for a gamepad.

One scripted HID service is allowed. It consumes one service slot and
`3 + 2 * report-count` entries from the characteristic and descriptor capacity.
For one report, set the attribute capacity to at least 5. Automatic input CCCDs
do not add entries to this count.

The returned handle list is:

```text
(service information report-map control-point report-1 reference-1 ...)
```

Each report contributes one Report characteristic and one Report Reference
descriptor. Input reports support read and notify. Output reports support read,
write, and write without response. Feature reports support read and acknowledged
write. Report values start as zero-filled arrays of their declared size.

Invalid argument structures or ranges raise `type_error`. Registration failures,
duplicate reports, mixed numbered and unnumbered reports, or insufficient
capacity raise `eval_error`.

### What is a report map?

The report map describes the **layout and meaning of report bytes**. For example,
a mouse map can define three button bits and two signed movement bytes. The
host reads that description to interpret later input reports.

The descriptor format and usage identifiers are standardized. There is no single
map for every keyboard, mouse, or gamepad, because devices can have different
buttons, axes, and report layouts. The format is the HID Report Descriptor used
by USB HID as well as Bluetooth HID.
[USB-IF HID specifications and tools](https://usb.org/hid) define the descriptor
format and usages. A complete BLE mouse reference is available in the
[Zephyr HID peripheral sample](https://docs.zephyrproject.org/latest/samples/bluetooth/peripheral_hids/README.html).

Use a known descriptor that matches the data your script sends, or create one
for your device. Copying a descriptor without also matching its payload format
will produce incorrect input on the host.

### Example: three-button mouse

Enable the server role, configure one service and at least five attributes, then
save and reboot. This example defines an unnumbered three-byte input report:
button bits, signed X movement, and signed Y movement.

```clj
(ble-set-name "VESC Mouse")
(ble-start-app)

; Remove all scripted services from a previous run.
(map ble-remove-service (reverse (ble-get-services)))

(def mouse-map [
    0x05 0x01       ; Usage Page: Generic Desktop
    0x09 0x02       ; Usage: Mouse
    0xa1 0x01       ; Application collection
    0x09 0x01       ; Usage: Pointer
    0xa1 0x00       ; Physical collection
    0x05 0x09       ; Usage Page: Button
    0x19 0x01       ; First button: 1
    0x29 0x03       ; Last button: 3
    0x15 0x00       ; Logical minimum: 0
    0x25 0x01       ; Logical maximum: 1
    0x75 0x01       ; One bit per button
    0x95 0x03       ; Three buttons
    0x81 0x02       ; Input: data, variable, absolute
    0x75 0x05       ; Five padding bits
    0x95 0x01       ; One padding field
    0x81 0x03       ; Input: constant padding
    0x05 0x01       ; Usage Page: Generic Desktop
    0x09 0x30       ; Usage: X
    0x09 0x31       ; Usage: Y
    0x15 0x81       ; Logical minimum: -127
    0x25 0x7f       ; Logical maximum: 127
    0x75 0x08       ; Eight bits per axis
    0x95 0x02       ; Two axes
    0x81 0x06       ; Input: data, variable, relative
    0xc0            ; End physical collection
    0xc0            ; End application collection
])

(ble-conf-adv nil)
(def hid-handles (ble-hid-add-service mouse-map '((0 1 3)) 0x03c2))
(def mouse-input (ix hid-handles 4))
```

Pair from the computer or phone after registering the service. Once the host
has established encryption and subscribed, send reports from the REPL or your
application:

```clj
(ble-attr-set-value mouse-input [0 10 0])  ; Move right by 10
(ble-attr-set-value mouse-input [1 0 0])   ; Press the left button
(ble-attr-set-value mouse-input [0 0 0])   ; Release all buttons
```

A negative displacement uses its signed-byte encoding, for example `0xf6` for
-10. Do not prepend a report ID. In BLE Report characteristics, the Report
Reference descriptor identifies the report.

### Sending and receiving reports

Send input with `ble-attr-set-value` on its report handle. The array length must
exactly match the declared report size. To transmit a notification, the host
must be connected, encrypted, and subscribed, and the payload must fit
`MTU - 3` bytes. Without a subscription, the call only updates the cached value.
A report declared larger than that packet limit cannot be sent as a single
input notification.

Read output and feature values with `ble-attr-get-value`, or enable
`event-ble-rx` to process writes from the host. Remote report writes must also
match the declared length exactly.

The HID Control Point accepts one byte: 0 suspends input notifications and 1
resumes them. Resuming does not automatically retransmit cached data. After
reconnection, the host must subscribe again, and the script must write a new
report to trigger transmission.

### Pairing, advertising, and protocol support

The helper requires encryption for HID characteristic reads and remote writes,
and enables bonding. It preserves the incoming server's configured PIN and
MITM policy. Encrypted VESC mode uses the configured server PIN. Other server
modes retain their existing policy rather than acquiring a new PIN requirement.

Default advertising includes the HID service UUID and appearance after the
service is registered. Custom advertising must include the required discovery
fields itself. Register all services before pairing. After changing the report
map or service layout, remove the old pairing on the host and pair again so it
discovers the new layout.

The helper implements **Report Protocol**: the host interprets reports using
the report map. It does not create Protocol Mode or Boot Keyboard/Mouse
characteristics, so **Boot Protocol is not supported**. Adding a keyboard report
map alone does not provide Boot Protocol support.

Battery and Device Information services are not added automatically. Add them
separately when required by the device profile.

### Connecting to an external HID device

The client API supplies GATT transport and pairing, but does not decode HID
reports into key, mouse, or controller events. The script must interpret them.

1. Connect and complete any required pairing. Handle `ble-passkey-action` if the
   peripheral requires PIN entry or numeric comparison.
2. Discover HID service `0x1812` and its characteristics.
3. Read Report Map characteristic `0x2a4b` with `ble-client-read-long`.
4. Discover Report characteristics `0x2a4d` and their descriptors.
5. Read Report Reference descriptors `0x2908` to identify report IDs and types.
6. Subscribe to input reports using their discovered CCCDs, UUID `0x2902`.
7. Decode `ble-notification` payloads according to the peer's report map.

Wait for each procedure's completion before starting the next. Keep one client
event consumer and monitor `ble-client-dropped` throughout discovery and use.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| A BLE extension is unavailable | Enable its server or client role, write the configuration, and reboot |
| Adding a service raises `eval_error` | Call `ble-start-app` first, check both capacities, and check for reserved or duplicate service UUIDs |
| A script fails when run a second time | Services and client connections survive Lisp reloads. Remove old services in reverse order and stop old queue consumers |
| The name does not change | Call `ble-set-name` before the first `ble-start-app`. Check custom advertising for a separately supplied name |
| A client request appears to succeed but nothing happens | Compare its return value with `t`, then check the completion event and status |
| A request returns a busy error | Wait for the connection's active GATT or security procedure, or for the pending connection attempt |
| No notifications arrive | Check the characteristic properties, discovered CCCD, successful subscription write, connection, and payload size |
| A long value is truncated | Use `ble-client-read-long` instead of an ordinary read |
| Data appears stale after an error | Local server writes update the cache before notification is attempted. Check the return value and peer state |
| Discovery results are missing | Monitor the drop counter, poll more frequently, and repeat discovery after recovering from event loss |
| A bonded peer cannot pair again | Remove its bond with `ble-client-security connection 'forget`, then reconnect |
| A computer rejects a changed HID device | Remove the computer's old pairing, register the final service layout, and pair again |
| A HID input write fails or has no effect | Check exact report length, report-map layout, encryption, subscription, Control Point state, and `MTU - 3` |

## Implementation and references

The behavior documented here is implemented in:

- [lispif_ble_extensions.c](lispif_ble_extensions.c): Lisp argument validation,
  extension availability, return values, and event layouts.
- [custom_ble.c](custom_ble.c) and [custom_ble.h](custom_ble.h): dynamic services,
  HID helper, client procedures, connection limits, and event buffering.
- [comm_ble.c](../comm_ble.c): shared NimBLE host, built-in VESC transport,
  incoming pairing policy, and preferred MTU.
- [Board build profiles](../../): compiled connection pools and bond persistence.

The server reserves CCCD resources before starting the shared host so
notification-capable services can be registered later. Client callbacks copy
data into a bounded FreeRTOS queue and leave Lisp allocation to the consumer.
These allocations depend on the configured server capacity and compiled BLE
connection pools.

Protocol references:

- [Bluetooth Assigned Numbers](https://www.bluetooth.com/specifications/assigned-numbers/)
  lists advertising types, service and characteristic UUIDs, appearance values,
  and company identifiers.
- [Bluetooth Attribute Protocol](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-62/out/en/host/attribute-protocol--att-.html)
  defines attribute sizes, ATT reads, writes, notifications, and indications.
- [USB-IF HID specifications and tools](https://usb.org/hid) define report
  descriptors and HID usages.
- [LispBM language reference](../lispBM/doc/lbmref.md) describes the language,
  byte arrays, and event handling used in the examples.
