# Documentation for the BLE interface for LispBM

## Introduction

The BLE interface provides a set of function allowing you to control control the
devices BLE behavior from LispBM scripts, allowing it to act as a BLE server. The library is quite limited at the
moment (it does not support encryption at all at the moment for instance). It
only supports the following actions:

1. Defining services with UUIDs and a list of characteristics with their own UUIDs.
2. Defining the access permissions of said charactersistics, including read, write,
   write no response, notify, and indicate.
3. Defining a list of charactersistic descriptors to assign to a characteristic.
4. Reading and writing the values of the characteristics and their descriptors.

## Configuring BLE Scripting

To make use of the BLE scripting library, you first need to configure the custom
config of the VESC in VESC Tool. This is done by first connecting your VESC to
VESC Tool and then going to **VESC Express** > **Bluetooth** and setting
**Bluetooth Mode** to **Enabled with Scripting**. Don't forget to write the
value after editing it! Note that this will disable your ability to connect
VESC Tool through Bluetooth. It is currently not possible to connect VESC Tool
through Bluetooth while BLE scripting is enabled. If this option isn't enabled,
the libraries extensions are simply not defined.

You also need to configure the amount of services, characteristics and
descriptors you plan to define. Internal arrays that contain these are allocated
at startup, so you need to define the sizes of these. This is done through the
**BLE Service Capacity** and **BLE Characteristic and Descriptor Capacity**
settings. The characteristic and descriptor capacity setting counts the sum of
the number characteristics and descriptors.

**Note**: These settings only take effect after a reboot, which can be achieved
by selecting **Terminal** > **Reboot**.

## The Library

### `ble-start-app`

```clj
(ble-start-app)
```

This starts the BLE server with the configured name, allowing other devices to
connect and you to create services.
This should be called once at the start of the program. It has no effect on
subsequent calls (currently, the only way to stop the BLE server is to power
cycle the VESC). It returns `true` the first time it is called and `nil` on
subsequent calls.

This function may throw an `eval_error` if some previous internal mechanism
has failed.

### `ble-set-name`

```clj
(ble-set-name name)
```

Configure the name that the BLE server advertises as the device's name. So
essentially, the devices name in the BLE system.

The name must not be longer than 30 characters. Providing a longer name will
throw an `eval_error`. In practice the limit is probably much shorter, since the
name is placed in a advertising packet together with other data which has a
limit of 31 bytes in total. (TODO: The precise mechanics of this need to be figured
out.)

This should be called *before* the BLE server has started (so before
[`ble-start-app`](#ble-start-app) is called). Then `true` is returned. Calling it afterwards has no effect
and returns `nil`.

### `ble-add-service`

```clj
(ble-add-service service-uuid characteristics)
```

Add a service with the given list of characteristics.

This function needs to be called after [`ble-start-app`](#ble-start-app) has been
called. Calling it before will throw an `eval_error`.

The service-uuid is given as a byte array representing either a 16-, 32-, or
128-bit UUID, so it should be 2, 4, or 16 bytes long. The order of the bytes is
big endian, which means that you write the bytes in the same order as you would
write it in text (so the UUID 4be24176-71ae-11ee-b962-0242ac120002 would be represented
by the lbm value `[0x4b 0xe2 0x41 0x76 0x71 ... 0x12 0x00 0x02]`).

This function returns a list of service, characteristic, and descriptor handles
in the order they were defined, with the service handle being first (See the
example below). This function can also return a `type_error` or `eval_error`.

#### `characteristics` Format

The characteristics list is defined by a list of associative lists, where each
assoc list defines a single characteristic. Assoc lists are lists with cons
cells as items. Where the car fields hold the lists keys, while the cdr fields
hold the keys respective values.

The valid entries are as follows:
- `'uuid`: The characteristic's UUID, same format as the service UUID.
- `'prop`: A list of any combination of the symbols `'prop-read`, `'prop-write`,
  `'prop-write-nr`, `'prop-indicate`, `'prop-notify`, which correspond to the
  access flags for this characteristic.
- `'max-len`: The maximum value length (in bytes) that can be written to this
  characteristic.
- `'default-value` \[optional\]: A byte array which will be set as the
  characteristics initial value. The length of this should not be longer than
  `max-len`.
- `'descr` \[optional\]: A list of descriptors to assign to this characteristic
  (see below).

The descriptor list follows a very similar format to the characteristic list,
except only the `'uuid`, `'max-len`, and `'default-value` entries are valid.

**Note**: If you configure `'prop` to include either of the flags `prop-indicate` or
`prop-notify`, you also need to include a client characteristic control (CCC)
descriptor. This is easily done by adding a characteristic with the 16-bit UUID
`[0x29 0x02]`, a max length of 2, and a initial value of `[0 0]` (see the example below).  

#### Example

```clj
(ble-add-service [0xbe 0xb5 0x48 0x3e] '(
    (
        (uuid . [0xbe 0xb5 0x48 0x4e])
        (prop . (prop-read prop-write prop-notify))
        (max-len . 100)
        (descr . (
            (
                (uuid . [0x29 0x02]) ; the ccc-uuid
                (max-len . 2)
                (default-value . [0 0])
            )
        ))
    )
))
> (40u 42u 43u)
```

This creates a service with the 32-bit UUID [0xbe 0xb5 0x48 0x3e] with a single
characteristic that has the 32-bit UUID [0xbe 0xb5 0x48 0x4e] (these UUIDs are
complete nonsense). Client devices are allowed to read, write the value of this
characteristic, which can at most be 100 bytes long.

Clients are also allowed to subscribe to notifications of changes to the value,
which is why we need to add the CCC descriptor to the characteristic. The value
of and behavior of this descriptor is handled automatically by the BLE server.
Note that we also could have defined any other characteristics if we wanted to.

This function then returns a list of handles. The service handle 40 comes first,
followed by the characteristic and descriptor handles 42 and 43 respectively.
These can be used with the other library functions.

### `ble-remove-service`

```clj
(ble-remove-service service-handle)
```

Remove the given service, along with the associated characteristics and
descriptors, freeing the resources in the process. `service-handle` should be a
number which was the first number in the list of handles returned by
[`ble-add-service`](#ble-add-service).

Due to technical limitations (that should probably be possible to remove),
services need to be removed in the reverse order they were created in. Say you
created two services, **Service A** and then **Service B**, in that order. To
remove them, you would have to first remove **Service B**, followed by
**Service A**. This is because the services are stored on a stack internally, so
you need to pop them in the order you usually would with a stack.

Example which remove the service with the handle 40, which was the service which was
created last:

```clj
(ble-remove-service 40)
> t
```

### `ble-attr-get-value`

```clj
(ble-attr-get-value attr-handle)
```

Get the current value of a characteristic or descriptor (referred to generally as
an attribute) as a byte array. `attr-handle` should be a number
returned by [`ble-add-service`](#ble-add-service).

Providing an invalid handle will throw an `eval_error`.

Example where the value of the attribute with the handle 42 is set to the
byte-array `"hello"`, and then queried:

```clj
(ble-attr-set-value 40 "hello")
(ble-attr-get-value 40)
> "hello"
```

### `ble-attr-set-value`

```clj
(ble-attr-set-value attr-handle value)
```

Set the value of the attribute to the provided value. `attr-handle` should be a
number that was returned by [`ble-add-service`](#ble-add-service) and `value` a byte
array.

Providing an invalid handle will throw an `eval_error`.

Setting the value of a characteristic automatically sends notifications or
indications to subscribing clients if those flags were set when the
characteristic was defined.

### `ble-get-services`

```clj
(ble-get-services)
```

Get the list of the handles of the currently active services, excluding any
services that have already been removed. The handles are returned in the order
they were created in. If you want to remove them, you would need to reverse the
list first!

Example that gets the currently active service handles:

```clj
(ble-get-services)
> (40u)
```

In this case, only a single service had been created.

### `ble-get-attrs`

```clj
(ble-get-attrs service-handle)
```

Get the list of characteristic and descriptor handles assigned to the given service
when it was created. `service-handle` should be a number that was returned by
[`ble-add-service`](#ble-add-service). Providing an invalid handle throws an
`eval_error`.

This function essentially returns the same list returned when `ble-add-service` was
first called, just without the first item (since that item would correspond to the service's
handle).

Example where the attribute handles of the service 40 are queried:

```clj
(ble-get-attrs 40)
> (42u 43u)
```

In this case, the service was defined with a single characteristic with a
descriptor that had the handles 42 and 43 respectively.

## Events
Too know when a client has written to a characteristic or descriptor, an LBM
event is fired. The semantics work the same as with
[normal events](https://github.com/vedderb/bldc/blob/master/lispBM/README.md#events).
The event message consists of a list of the form
`('event-ble-rx handle data)`, where the `handle` is the handle number of the
attribute's that was written to, and the `data` is the byte-array that was
written by the client.

Here is an example of a setup that catches and logs these events, printing the
values as strings:
```clj
(defun proc-ble-data (handle data) {
    (print (str-merge
        "Value "
        (to-str data)
        " written to attribute handle "
        (str-from-n handle)
    ))
})

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-ble-rx (? handle) (? data)) (proc-ble-data handle data))
            (_ nil) ; Ignore other events
)))

(event-register-handler (spawn event-handler))
(event-enable 'event-ble-rx)
```