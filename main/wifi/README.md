# Documentation of the WiFi and TCP interface for LispBM

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
[`wifi-connect`](#wifi_connect) or [`wifi-disconnect`](#wifi-disconnect). This
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
form `(ssid rssi channel)`.

The same network SSID might appear multiple times for reasons I'm not entirely
sure of at the moment. Just make sure you're aware of it!

Example that scans the available network on all WiFi channels:
```clj
(wifi-scan-networks 0.12 0)
> (("Network 1" -64 6u) ("Network 2" -64 11u) ("Network 1" -75 1u) ("Network 3 2.4Gh" -86 1u) ("Network 1" -87 6u))
```
(The SSIDs are of course fictional)

### `wifi_connect`

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
unfortunately isn't that reliable, so you may want to try a second time when
receiving `nil` to be sure.

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

