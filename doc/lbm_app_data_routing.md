# App-data reply routing

The legacy `event-data-rx` event and `recv-data` keep their existing formats. By default, `send-data` uses the last command connection.

For delayed replies to the requesting client, enable `event-data-rx-source`. It takes precedence over `event-data-rx` and delivers `(event-data-rx-source source . data)`, where `source` is an opaque token. Bind it with `(app-data-reply source)` before handling the request, and clear it with `(app-data-reply 0)` afterward.

```lisp
((event-data-rx-source (? source) . (? data)) {
    (var result (trap {
        (app-data-reply source)
        (handle-request data)
    }))
    (app-data-reply 0)
    ; Inspect result for a trapped error here.
})
```

While bound, default `send-data` calls in this evaluator context use that source. Other contexts and explicit interface selections are unaffected. Clear the binding after both success and failure; it is also cleared when the context finishes or Lisp restarts. A disconnected BLE source is invalidated, and pending replies to it are dropped rather than falling back to another client. Tokens must not be saved in startup images or reused across Lisp restarts. Source and context tables each hold 16 entries; requests without an available source entry are dropped.
