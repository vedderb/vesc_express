# Documentation of the WiFi and TCP interfaces for LispBM

## Introduction

The VESC Express firmware provides interfaces to control WiFi connectivity , as
well as APIs to allow the VESC to function as a TCP client, including managing
TCP connections and sending and receiving data over said connections.

## Configuring WiFi and TCP scripting

To be able to use the APIs, the VESC needs to be configured to be in Station
Mode. This is done by first connecting your VESC to VESC tool and then going to
**VESC Express** > **WiFi** and setting **WiFi Mode** to **Station**. Don't
forget to write the value and rebooting after editing it for the change to take
effect!

All of these extensions throw an `eval_error` if the VESC is not configured to be in
**Station Mode**.

**Note** that the VESC can only be connected to a single WiFi network at a time, so
**if you're using WiFi connectivity to connect to VESC tool, your connection will
be broken if a script changes or disconnects from the current network** using
[`wifi-connect`](#wifi-connect) or [`wifi-disconnect`](#wifi-disconnect). This
also means that the `wifi-connect` extension has the same behavior as
configuring the **Station Mode SSID** and **Station Mode Key** settings.

## Overview

The functions of the WiFi API are prefixed with `wifi-` and include the
following capabilities:
- Scaning for nearby WiFi networks.
- Connecting to and disconnecting from a WiFi network.
- Querying the current connection status.

**Note**: The
[ESP-NOW](https://github.com/vedderb/bldc/blob/master/lispBM/README.md#esp-now)
API unfortunately also has some extensions with the `wifi-` prefix. These don't
really make sense to use while using this WiFi API (apart from maybe
[`wifi-get-chan`](https://github.com/vedderb/bldc/blob/master/lispBM/README.md#wifi-get-chan)),
so you can ignore them for the purposes of this document.

The functions of the TCP API are similarly prefixed with `tcp-` and include the
following capabilities:
- Opening and closing TCP sockets.
- Checking the status of a TCP socket.
- Sending and receiving data over an open TCP socket.

The entire TCP API can only be used by a single LispBM thread at a time, and
will throw an `eval_error` when calling any functions from different threads
at the same time.

## The WiFi Library

### `wifi-scan-networks`

```clj
(wifi-scan-networks [scan-time] [channel] [show-hidden])
```

Perform a scan of the available nearby network SSIDs. All parameters are
optional.
- `scan-times`: The time in seconds to scan for (at least) *per channel*. So if
  `channel` is set to 0 (all channels), you would need to multiply this value by
  14 to get a (very) approximate duration of the entire scan.
  **Default: `0.12`**
- `channel`: The WiFi channel to perform the scan on. The value should be an
  integer from 1 to 14 (inclusive), and setting it to 0 performs the scan on all
  channels (the scan will then also take 14 times as long). **Default: `0`**
- `show-hidden`: Include hidden networks in the search. (I'm honestly not
  certain of the exact behavior of this, so it's pretty much untested. Feel
  free to ignore this option.) **Default: `false`**

The found list of networks is returned as a list of network tuples, each of the
form `(ssid rssi channel ftm-responder (mac-addr))`.

The same network SSID might appear multiple times for reasons I'm not entirely
sure of at the moment. Just make sure you're aware of it!

Example that scans the available network on all WiFi channels:
```clj
(loopforeach i (wifi-scan-networks) (print i))
> ("ESP_0927BD" -28 7 1 (104 103 37 9 39 189))
> ("vescnet" -55 7 0 (8 191 184 96 152 48))
> ("ESP_89D4D9" -85 7 0 (128 101 153 137 212 217))
```
(The SSIDs are of course fictional)

### `wifi-connect`

```clj
(wifi-connect ssid password)
```

Connect to the specificed WiFi network. `ssid` should be a string and
`password` should be a string or `nil`. Pass `nil` if the network is open and
doesn't have a password.

On success `true` is returned. The VESC is then fully connected to the network
and you're able to immediately open TCP connections using
[`tcp-connect`](#tcp-connect). The symbol `'wrong-password` is returned if the
password was incorrect or the SSID didn't exist. Other errors result in `nil`
being returned. The mechanism for detecting authentication failures
unfortunately isn't that reliable, so you may want to retry when receiving `nil`
to be sure.

Note that a negative returned result doesn't always mean that the VESC isn't
connected. See [`wifi-auto-reconnect`](#wifi-auto-reconnect).

Example where we connect to the network 'Example 1':
```clj
(wifi-connect "Example 1" "wordpass")
> t

(wifi-status)
> connected
```

### `wifi-disconnect`

```clj
(wifi-disconnect)
```

Disconnect from the currently connected network, leaving the VESC entirely
unconnected. This function always returns `true`.

Example where we were connected to a network

```clj
(print (wifi-status))
> connected

(wifi-disconnect)

(print (wifi-status))
> disconnected
```

### `wifi-status`

```clj
(wifi-status)
```

Query the current network connection status. The status is returned as one of
the following symbols: `'connected`, `'connecting`, or `'disconnected`. The
difference between the `'connecting` and `'disconnected` states is honestly kind
of ill-defined currently...
However, `'connected` always means the VESC is connected to a WiFi network and
ready for TCP connections to be opened.

This function gives the same result as entering the `hw_status` command into the
VESC Terminal in VESC Tool and looking at the **WIFI Connected** and
**WIFI Connecting** fields.

Example where the VESC is currently connected to a network:
```clj
(wifi-status)
> connected
```

### `wifi-auto-reconnect`

```clj
(wifi-auto-reconnect [should-reconnect])
```

Configure if the internal WiFi event listener should automatically attempt to
reconnect to the currently configured WiFi network on disconnects.
`should-reconnect` is optional and should be a boolean value (containing `nil`
or `true`). If it isn't passed, this extension just reports the current value
without modifying it. The previous value is returned in either case. This
setting is configured to be `true` on startup.

Automatically reconnecting is normally the behavior that you'd want if using
WiFi to connect with VESC Tool without using any of these APIs, since you'd then
want it to try to reconnect at all costs. But if you're connecting to WiFi
networks with LBM scripts, this can produce strange behavior. Consider the case
where connecting to a network using [`wifi-connect`](#wifi-connect), and it
failed for some unknown reason. The `wifi-connect` extension would then return
`nil`, while the internal event loop attempts to reconnect. If it then succeeds
on the second try, you've got an inconsistency between the result reported by
`wifi-connect` and the actual connection status. **In conclusion: when you
disable automatic reconnections it is guaranteed that if `wifi-connect` returns
`nil` or `wrong-password`, then the VESC isn't connected to a WiFi network, and
never will be unless the lbm code takes action.**

If you disable automatic reconnecting it *is crucial* that you have set up an
event listener that listens for [disconnect events](#events) and takes the
necessary actions (such as manually reconnecting). Note that you should still
have one set up with automatic connections enabled, since there are still
scenarios where the internal wifi module might not reconnect, such as when it
thinks that wrong credentials were passed. This event *should* only happen when
caused by a call to `wifi-connect`, which would have notified the lbm code
either way, *but* you should still be aware of this as happening randomly as a
remote possibility to ensure a robust application.

TL;DR: it is recommended that you set this to false, and that you setup an event
listener to handle random disconnects.

Example where we disable automatic reconnections at startup (note the return
value).
```clj
(wifi-should-reconnect nil)
> t
```

### `wifi-ftm-measure`

```clj
(wifi-ftm-measure peer channel [debug-print])
```

Fine Time Measurement (FTM) can be used to calculate the distance to another WiFi-device by calculating the round trip time (RTT). The argument peer is a list with the mac-address of the other device, the argument channel is the primary WIFI-channel of the other device and the optional argument debug-print can be set to true to print debug information in the REPL if this function fails. On success the distance to the other device is returned in centimeters and on failure nil is returned.

Note that the other device must support ftm-responder for this command to work. VESC Express-devices with the latest firmware should support that when WiFi or ESP-NOW is running.

Example:

```clj
(print (wifi-ftm-measure '(104 103 37 9 39 189) 1))
> 150
; Here the result 150 means that the device with the mac-address (104 103 37 9 39 189) is 150 cm away.
```

 ---
 
## The TCP Library

### `tcp-connect`

```clj
(tcp-connect dest port)
```

Open a new tcp socket connected to the specificed `dest` and `port`. `dest`
must be a string either containing a hostname (like `"subdomain.example.com"`) or
an IPv4 address in the usual dot notation (like `"127.0.0.1"`). IPv6 addresses are
*not* supported. `port` should be an integer.

The created socket is returned on success, which is already connected to the
host ready for sending and receiving data. If the given hostname can't be
resolved, the symbol `'unknown_host` is returned. If the connection failed for
other reasons (including that there were no free sockets left), `nil` is
returned.

The socket must be closed at some point with [`tcp-close`](#tcp-close) for the
allocated resources to be freed, even if the remote has already disconnected.
Restarting the VESC has the same behavior as closing all open sockets (TODO:
verify this).

Example where we open a connection with the server under `example.com`.
```clj
(tcp-connect "example.com" 80)
> 61
```
`example.com` is a real hostname [maintained by
IANA](https://www.iana.org/help/example-domains), so the example should work
without modification. :)

### `tcp-close`

```clj
(tcp-close socket)
```

Close a tcp connection. `socket` should be an integer (as returned from
[`tcp-connect`](#tcp-connect)).

`true` is returned if the socket was recognized and closed successfully and
`nil` is returned if the provided socket didn't exist or was already closed.

Note that you still need to call this when the server has already disconnected
and the socket is unusable.

Example:
```clj
(def socket (tcp-connect "example.com" 80))
(tcp-close socket)
> t
```

### `tcp-status`

```clj
(tcp-status socket)
```

Query the connection status of the provided socket. One of the following symbols
is returned:
- `'connected`: The socket is connected to the remote, and it's possible to send
  and receive data.
- `'disconnected`: The remote has disconnected, but the socket has not been
  closed yet.
- `nil`: The socket wasn't valid (it didn't exist or was already closed).

When `'disconnected` is returned the only usefull action left is to close it.

Internally, this function works very similarly to [`tcp-recv`](#tcp-recv), only
difference being that this one doesn't consume the internal recv buffer. Both
functions detect disconnected remotes equally well. So if you're going to call
`tcp-recv`, calling `tcp-status` before would be unnecessary.

Example:
```clj
(def socket (tcp-connect "example.com" 80))
(tcp-status socket)
> connected
```

### `tcp-send`

```clj
(tcp-send socket data)
```

Send the byte-array `data` over the specified TCP socket.

When sent successfully, `true` is returned. If the remote has already
disconnected, `'disconnected` is returned, and `nil` is returned if the provided
socket wasn't valid or some other network error occurred.

Example that sends the string `"example data"` (which is just a byte-array) over the TCP socket 61:
```clj
(tcp-send 61 "example data")
> t
```

### `tcp-recv`

```clj
(tcp-recv socket max-len [timeout] [as-str])
```

Receive up to `max-len` bytes from the TCP connection under the specified
`socket`. The arguments `timeout` and `as-str` are optional. `timeout` should be
a number or `nil` (**Default: `1.0`**). When `nil` is passed only data which is
currently available will be returned, otherwise this function blocks for at
least `timeout` seconds for data to arrive. `as-str` should be a bool (the
symbols `true` or `false`). If set to true, an additional null byte is appended
to any received data to make sure that it can be processed as a string, even if
the current call to recv only received half of a received string
(**Default: `true`**).

If no data was received, either because none arrived within the specified
timeout period, or because none was available this instant when the timeout was
disabled, the symbol `'no-data` is returned.
If the data was received, a byte-array containing the received data is returned.
If the remote has closed the connection, the symbol `'disconnected` is returned.
Otherwise when providing an invalid socket or when other unspecified network
errors occur, `nil` is returned.

**Tip**: If you're using this function in the REPL, the result might not get
printed if the function takes too long to return (such as with a timeout of a
second). You therefore might want to wrap the call in a print function to make
sure it's still printed. Example: `(print (tcp-recv socket 100))`

Example where we make an HTTP request to `example.com`. This is a real hostname
[maintained by IANA](https://www.iana.org/help/example-domains), so go ahead and
try it yourself! :)

```clj
(def socket (tcp-connect "example.com" 80))
(print socket)
(tcp-send socket "GET / HTTP/1.1\r\nHost: example.com\r\nConnection: Close\r\n\r\n")

(print (tcp-recv socket 200))
> "HTTP/1.1 200 OK\r\nAge: 197780\r\nCache-Control: max-age=604800\r\nContent-Type: text/html; charset=UTF-8\r\nDate: Sun, 12 Nov 2023 16:40:09 GMT\r\nEtag: \"3147526947+ident\"\r\nExpires: Sun, 19 Nov 2023 16:40:09 GMT\r\nLast-Modified: Thu, 17 Oct 2019 07..." ; the string has been shortened
```

### `tcp-recv-to-char`

```clj
(tcp-recv-to-char socket max-len terminator [timeout] [as-str] [return-on-disconnect])
```

Receive up to `max-len` bytes from the TCP connection under the specified
`socket`, until the given `terminator` byte is encountered. The terminator
character is included in the returned buffer.

Optional arguments:
- `timeout`: How long to wait for data to arrive at least in seconds. **Note**
  that unlike [`tcp-recv`](#tcp-recv), this extension does not support passing
  `nil`, and does therefore not support receiving only the currently available
  data instanteniously. (**Default: `1.0`**)
- `as-str`: If set to true, an additional null byte is appended to any returned
  data to make sure that it can be processed as a string (strings in LBM are
  just byte arrays with a terminating null byte). If false is passed, the
  literal received bytes is just returned. Should be a bool (the symbols `true`
  or `false`). (**Default: `true`**)
- `return-on-disconnect`: In the case that the connection is broken in the
  middle of sending a string before the terminating character is received, the
  bytes that were received up until that point are normally thrown away and the
  symbol `'disconnected` is instead returned. If passing true, these bytes that
  were part of the broken message are then still returned. If no data had been
  received yet before disconnecting, `'disconnected` is returned regardless of
  this value. **Note** that this is a pretty rare scenario, since the connection
  would have to break while the remote was still sending data. If the remote
  simply disconnected after having sent the complete message, the received data
  is kept in an internal buffer and the connection is not considered
  disconnected until this data has been read by this extension. Should be a bool
  (the symbols `true` or `false`). (**Default: `false`**)

If the buffer is filled before encountering the terminator, the full buffer is
just returned without the terminator, so you might want to call this a second
time to receive the full message in that case.

Example where the server on port 61 had sent the message `"Hello\0world"` and we
receive the first message terminated by a zero byte:
```clj
(tcp-recv-to-char 61 100 0)
> "Hello"
```

Example where we receive the first line of an HTTP request. This example should
be runnable without any modifications, as example.com is a real domain:
```clj
(def char-newline 10b)

(def socket (tcp-connect "example.com" 80))
(print socket)
(tcp-send socket "GET / HTTP/1.1\r\nHost: example.com\r\nConnection: Close\r\n\r\n")

(print (tcp-recv-to-char socket 200 char-newline))
> "HTTP/1.1 200 OK\r\n"
```

## Events
This module defines the event `event-wifi-disconnect`, which is fired whenever
the VESC has disconnected from the WiFi network **and the internal WiFi module
has decided to not automatically attempt to reconnect**. This means that this
event will fire more often whenever automatic reconnection has been disabled
using [`wifi-auto-reconnect`](#wifi-auto-reconnect). If this event is received,
it's guaranteed (i hope...) that the VESC is not connected and will never
reconnect unless the lbm code takes action, *regardless* of the current setting
of `wifi-auto-reconnect`.

The semantics of this work the same as with
[normal events](https://github.com/vedderb/bldc/blob/master/lispBM/README.md#events).
The event message is of the form
`('event-wifi-disconnect reason from-extension)`, where `from-extension` is a
boolean value set to `true` if this disconnect event occurred during a call to
[`wifi-connect`](#wifi-connect), in which case the thread that called
`wifi-connect` has already been notified of the failure to connect. So if
`false` is passed this disconnect event occurred out of the blue, like if the
connected WiFi network suddenly went offline. `reason` is an integer directly
corresponding to one of the internal WiFi reason codes, as documented in [this
table in the ESP documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-reason-code).
The exact semantics of when these occur is not clear, so you can just ignore
this value unless you know what you're doing.

Here is an example of a setup that catches this event and automatically
reconnects:

```clj
(def wifi-ssid "Example network")
(def wifi-password "wordpass")

(defun proc-wifi-disconnect (reason from-extension) {
    (print (str-merge
        "wifi disconnected, reason: "
        (str-from-n reason)
        ", from-extension: "
        (to-str from-extension)
    ))
    
    (wifi-connect wifi-ssid wifi-password)
})

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-wifi-disconnect (? reason) (? from-extension))
                (proc-wifi-disconnect reason from-extension)
            )
            (_ nil) ; Ignore other events
        )
    )
)

(wifi-should-reconnect nil)
(event-register-handler (spawn event-handler))
(event-enable 'event-wifi-disconnect)
```
