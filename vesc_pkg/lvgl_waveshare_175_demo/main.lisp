@const-start

; Eurostile display font subsets for the dashboard values and units.
(import "fonts/eurostile-100.bin" 'font-speed-data)
(import "fonts/eurostile-40.bin" 'font-large-data)
(import "fonts/eurostile-30.bin" 'font-medium-data)
(import "fonts/eurostile-20.bin" 'font-small-data)

; Dashboard icons stored as RGB565 plus an A8 alpha plane.
(import "assets/battery.lvim" 'image-battery-data)
(import "assets/controller.lvim" 'image-controller-data)
(import "assets/motor.lvim" 'image-motor-data)
(import "assets/eco.lvim" 'image-eco-data)
(import "assets/race.lvim" 'image-race-data)
(import "assets/blank.lvim" 'image-blank-data)
; LVGL 9.5 enum values used by the narrow Express bridge.
(def align-top-mid 2)
(def align-bottom-mid 5)
(def align-left-mid 7)
(def align-right-mid 8)
(def align-center 9)
(def text-align-center 2)
(def text-align-right 3)
(def arc-mode-normal 0)
(def arc-mode-reverse 2)
(def size-content -1) ; translated to LV_SIZE_CONTENT by the native bridge

(def color-black 0x000000)
(def color-white 0xFFFFFF)
(def color-muted 0x7A7A7A)
(def color-gold 0xFFD700)
(def color-track 0x202020)
(def color-green 0x06A000)
(def color-red 0xFF2020)

; Display settings are owned by this package and persisted in LispBM EEPROM.
; Binary App UI protocol: "VD", version, opcode, request id, payload.
(def settings-protocol-magic-0 0x56)
(def settings-protocol-magic-1 0x44)
(def settings-protocol-version 1)
(def settings-op-get 1)
(def settings-op-set 2)
(def settings-op-get-profiles 3)
(def settings-op-save-profile 4)
(def settings-op-commit-profile 5)
(def settings-schema-magic 17501)
(def settings-eeprom-schema 0)
(def settings-eeprom-brightness 1)
(def settings-eeprom-speed-unit 2)
(def settings-eeprom-power-unit 3)
(def default-brightness 80)
(def default-speed-unit 0) ; 0 = km/h, 1 = mph
(def default-power-unit 0) ; 0 = watts, 1 = battery input amps

; Three package-owned temporary motor-controller profiles. Values are stored
; in display-friendly units and converted by the QML App UI before it invokes
; COMM_SET_MCCONF_TEMP_SETUP on the selected VESC controller.
; Record: brake %, drive %, reverse km/h x10, forward km/h x10,
;         minimum duty %, maximum duty %, regen watts, drive watts.
(def profile-schema-magic 17502)
(def profile-eeprom-schema 16)
(def profile-eeprom-selected 17)
(def profile-eeprom-enabled 18)
(def profile-eeprom-data 32)
(def profile-count 3)
(def profile-field-count 8)
(def profile-record-size 12)
(def profile-response-size 46)
(def command-forward-can 34)
(def command-set-mcconf-temp-setup 49)
(def command-get-mcconf-temp 91)
(def guardian-command-none 0)
(def guardian-command-set 1)
(def guardian-command-get 2)
(def guardian-verify-seconds 15.0)
(def guardian-retry-seconds 3.0)
(def guardian-response-timeout-seconds 1.5)
(def make-profile-default-values (fn () {
    ; Constructed when main starts so setix receives a mutable heap list,
    ; rather than a list placed in flash by the surrounding constant block.
    (list
        100 100 990 990 5 95 9999 9999
        100 100 990 990 5 95 9999 9999
        100 100 990 990 5 95 9999 9999)
}))

; Live telemetry policy. VESC Express receives standard VESC CAN status
; frames and exposes them to LispBM through the canget-* functions below.
; The first controller with a fresh status-1 frame is selected automatically.
(def telemetry-stale-seconds 1.0)
(def telemetry-filter-alpha 0.25)

; canget-rpm returns electrical RPM. These package-local defaults can be
; changed without reflashing VESC Express. A later settings package can
; persist these values in LispBM.
(def motor-pole-pairs 14.0)       ; 28 motor poles / 2
(def drive-gear-ratio 1.0)
(def wheel-diameter-m 0.5)
(def pi 3.141592653589793)

; Used only when no fresh VESC BMS state-of-charge value is available.
(def battery-min-voltage 39.0)    ; 13S x 3.0 V
(def battery-max-voltage 54.6)    ; 13S x 4.2 V
(def controller-temp-arc-max 100)
(def motor-temp-arc-max 120)

(def make-label (fn (parent font text width height align x y color text-align) {
    (var label (lv-label-create parent))
    (lv-object-set-size label width height)
    (lv-object-align label align x y)
    (lv-label-text label text)
    (lv-label-style label color font text-align)
    label
}))

(def make-image (fn (parent resource align x y color) {
    (var image (lv-image-create parent))
    (lv-image-set-src image resource)
    (lv-object-align image align x y)
    (lv-image-recolor image color 255)
    image
}))

(def make-arc (fn (parent size maximum start-angle end-angle mode width) {
    (var arc (lv-arc-create parent))
    (lv-object-set-size arc size size)
    (lv-object-align arc align-center 0 0)
    (lv-arc-config arc 0 maximum start-angle end-angle mode)
    (lv-arc-style arc color-track color-white width)
    arc
}))

(def clamp (fn (value minimum maximum) {
    (if (< value minimum)
        minimum
        (if (> value maximum) maximum value))
}))

(def round-int (fn (value) {
    (to-i (if (>= value 0.0) (+ value 0.5) (- value 0.5)))
}))

(def msg-fresh (fn (id message-number) {
    (if (< id 0)
        nil
        {
            (var age (can-msg-age id message-number))
            (and (number? age) (< age telemetry-stale-seconds))
        })
}))

(def find-live-controller (fn (devices) {
    (if devices
        {
            (var id (car devices))
            (if (msg-fresh id 1)
                id
                (find-live-controller (cdr devices)))
        }
        -1)
}))

(def reset-filters (fn () {
    (setq speed-filter-valid nil)
    (setq battery-filter-valid nil)
    (setq controller-filter-valid nil)
    (setq motor-filter-valid nil)
    (setq power-filter-valid nil)
}))

(def select-controller (fn () {
    (var selected (find-live-controller (can-list-devs)))
    (if (!= selected controller-id) {
        (setq controller-id selected)
        (reset-filters)
        (print (str-merge "LVGL telemetry controller "
            (str-from-n controller-id "%d")))
    })
}))

(def erpm-to-kmh (fn (erpm) {
    (var motor-rpm (/ erpm motor-pole-pairs))
    (var wheel-rpm (/ motor-rpm drive-gear-ratio))
    (* (/ wheel-rpm 60.0) pi wheel-diameter-m 3.6)
}))

(def vin-to-percent (fn (vin) {
    (clamp (* (/ (- vin battery-min-voltage)
                 (- battery-max-voltage battery-min-voltage)) 100.0)
           0.0 100.0)
}))

(def settings-write-if-changed (fn (address value) {
    (var current-value (eeprom-read-i address))
    (if (or (eq current-value nil) (!= current-value value))
        (eeprom-store-i address value))
}))

(def settings-store (fn () {
    (settings-write-if-changed settings-eeprom-schema settings-schema-magic)
    (settings-write-if-changed settings-eeprom-brightness display-brightness)
    (settings-write-if-changed settings-eeprom-speed-unit speed-unit)
    (settings-write-if-changed settings-eeprom-power-unit power-unit)
}))

(def settings-load (fn () {
    (if (eq (eeprom-read-i settings-eeprom-schema) settings-schema-magic) {
        (setq display-brightness
            (clamp (or (eeprom-read-i settings-eeprom-brightness)
                       default-brightness) 1 100))
        (setq speed-unit
            (if (= (eeprom-read-i settings-eeprom-speed-unit) 1) 1 0))
        (setq power-unit
            (if (= (eeprom-read-i settings-eeprom-power-unit) 1) 1 0))
    } {
        (setq display-brightness default-brightness)
        (setq speed-unit default-speed-unit)
        (setq power-unit default-power-unit)
        (settings-store)
    })
}))

(def profile-value-index (fn (profile field) {
    (+ (* profile profile-field-count) field)
}))

(def profile-value (fn (profile field) {
    (ix profile-values (profile-value-index profile field))
}))

(def profile-clamp-field (fn (field value) {
    (if (or (= field 0) (= field 1) (= field 4) (= field 5))
        (clamp value 0 100)
        (if (or (= field 2) (= field 3))
            (clamp value 0 3000)
            (clamp value 0 30000)))
}))

(def profile-set-value (fn (profile field value) {
    (var index (profile-value-index profile field))
    (var safe-value (profile-clamp-field field value))
    (setix profile-values index safe-value)
    (settings-write-if-changed (+ profile-eeprom-data index) safe-value)
}))

(def profile-store-defaults (fn () {
    (settings-write-if-changed profile-eeprom-selected selected-profile)
    (settings-write-if-changed profile-eeprom-enabled profile-enabled)
    ; Missing profile fields already mean the defaults held in RAM. Writing
    ; only this small header avoids a burst of 24 NVS transactions on first
    ; package startup. The schema marker is written last for atomic recovery.
    (settings-write-if-changed profile-eeprom-schema profile-schema-magic)
}))

(def profile-load (fn () {
    (setq selected-profile 0)
    (setq profile-enabled 0)
    (if (eq (eeprom-read-i profile-eeprom-schema) profile-schema-magic) {
        (setq selected-profile
            (clamp (or (eeprom-read-i profile-eeprom-selected) 0)
                   0 (- profile-count 1)))
        (setq profile-enabled
            (if (= (or (eeprom-read-i profile-eeprom-enabled) 0) 1) 1 0))
        (looprange i 0 (* profile-count profile-field-count) {
            (var stored-value (eeprom-read-i (+ profile-eeprom-data i)))
            (if (number? stored-value)
                (setix profile-values i
                    (profile-clamp-field (mod i profile-field-count)
                                         stored-value)))
        })
        (settings-write-if-changed profile-eeprom-enabled profile-enabled)
    } (profile-store-defaults))
}))

(def profile-resource (fn (profile) {
    ; Load only the icon family that is actually shown. Profile startup must
    ; never consume enough LVGL resource RAM to block the dashboard itself.
    (if (= profile 0) {
        (if (= resource-eco 0)
            (setq resource-eco (lv-image-load image-eco-data)))
        resource-eco
    } {
        (if (= resource-race 0)
            (setq resource-race (lv-image-load image-race-data)))
        resource-race
    })
}))

(def profile-show-icon (fn (profile) {
    (var image-resource (profile-resource profile))
    (var image-color (if (= profile 0)
        color-green
        (if (= profile 1) color-white color-red)))
    (var image-x (if (= profile 0) 55 63))
    (if (= image-profile 0)
        (setq image-profile
            (make-image screen image-resource align-left-mid image-x 0 image-color))
        {
            (lv-image-set-src image-profile image-resource)
            (lv-object-align image-profile align-left-mid image-x 0)
            (lv-image-recolor image-profile image-color 255)
        })
}))

(def profile-send-response (fn (opcode request-id status) {
    (var response (array-create profile-response-size))
    (bufset-u8 response 0 settings-protocol-magic-0)
    (bufset-u8 response 1 settings-protocol-magic-1)
    (bufset-u8 response 2 settings-protocol-version)
    (bufset-u8 response 3 opcode)
    (bufset-u8 response 4 request-id)
    (bufset-u8 response 5 status)
    (bufset-u8 response 6 (if (< controller-id 0) 255 controller-id))
    (bufset-u8 response 7 selected-profile)
    (looprange profile 0 profile-count {
        (var offset (+ 8 (* profile profile-record-size)))
        (bufset-u8 response offset (profile-value profile 0))
        (bufset-u8 response (+ offset 1) (profile-value profile 1))
        (bufset-u16 response (+ offset 2) (profile-value profile 2))
        (bufset-u16 response (+ offset 4) (profile-value profile 3))
        (bufset-u8 response (+ offset 6) (profile-value profile 4))
        (bufset-u8 response (+ offset 7) (profile-value profile 5))
        (bufset-u16 response (+ offset 8) (profile-value profile 6))
        (bufset-u16 response (+ offset 10) (profile-value profile 7))
    })
    (bufset-u8 response 44 profile-enabled)
    (bufset-u8 response 45
        (if (and (= profile-enabled 1)
                 (= applied-profile selected-profile)
                 (= guardian-synced-controller controller-id)
                 (msg-fresh controller-id 1)) 1 0))
    (send-data response)
}))

(def profile-save-request (fn (data request-id) {
    (if (>= (buflen data) 18) {
        (var profile (bufget-u8 data 5))
        (if (< profile profile-count) {
            (profile-set-value profile 0 (bufget-u8 data 6))
            (profile-set-value profile 1 (bufget-u8 data 7))
            (profile-set-value profile 2 (bufget-u16 data 8))
            (profile-set-value profile 3 (bufget-u16 data 10))
            (profile-set-value profile 4 (bufget-u8 data 12))
            (profile-set-value profile 5 (bufget-u8 data 13))
            (profile-set-value profile 6 (bufget-u16 data 14))
            (profile-set-value profile 7 (bufget-u16 data 16))
            (profile-send-response 0x84 request-id 0)
        } (profile-send-response 0x84 request-id 2))
    } (profile-send-response 0x84 request-id 1))
}))

(def profile-commit-request (fn (data request-id) {
    (if (>= (buflen data) 6) {
        (var profile (bufget-u8 data 5))
        (if (< profile profile-count) {
            ; The Express-resident guardian owns the actual VESC write and
            ; responds only after the controller acknowledgement arrives.
            (if (not profile-ready)
                (profile-send-response 0x85 request-id 5)
                (if (or guardian-busy (>= guardian-request-profile 0))
                    (profile-send-response 0x85 request-id 4)
                    (if (not (msg-fresh controller-id 1))
                        (profile-send-response 0x85 request-id 3)
                        {
                            (setq guardian-request-profile profile)
                            (setq guardian-request-id request-id)
                        })))
        } (profile-send-response 0x85 request-id 2))
    } (profile-send-response 0x85 request-id 1))
}))

(def settings-apply (fn () {
    (lv-display-brightness display-brightness)
    (lv-label-text label-speed-unit (if (= speed-unit 0) "kmh" "mph"))
    (lv-label-text label-power-unit (if (= power-unit 0) "watts" "amps"))
    ; The two power modes use different scales, so restart that filter.
    (setq power-filter-valid nil)
}))

(def settings-send-response (fn (opcode request-id status) {
    (var response (array-create 9))
    (bufset-u8 response 0 settings-protocol-magic-0)
    (bufset-u8 response 1 settings-protocol-magic-1)
    (bufset-u8 response 2 settings-protocol-version)
    (bufset-u8 response 3 opcode)
    (bufset-u8 response 4 request-id)
    (bufset-u8 response 5 status)
    (bufset-u8 response 6 display-brightness)
    (bufset-u8 response 7 speed-unit)
    (bufset-u8 response 8 power-unit)
    (send-data response)
}))

(def settings-receive (fn (data) {
    (if (and (>= (buflen data) 5)
             (= (bufget-u8 data 0) settings-protocol-magic-0)
             (= (bufget-u8 data 1) settings-protocol-magic-1)
             (= (bufget-u8 data 2) settings-protocol-version)) {
        (var opcode (bufget-u8 data 3))
        (var request-id (bufget-u8 data 4))
        (if (= opcode settings-op-get)
            (settings-send-response 0x81 request-id 0)
            (if (= opcode settings-op-set) {
                (if (>= (buflen data) 8) {
                    (setq display-brightness
                        (clamp (bufget-u8 data 5) 1 100))
                    (setq speed-unit
                        (if (= (bufget-u8 data 6) 1) 1 0))
                    (setq power-unit
                        (if (= (bufget-u8 data 7) 1) 1 0))
                    (settings-store)
                    (settings-apply)
                    (settings-send-response 0x82 request-id 0)
                } (settings-send-response 0x82 request-id 1))
            }
            (if (= opcode settings-op-get-profiles)
                (profile-send-response 0x83 request-id
                    (if profile-ready 0 5))
                (if (= opcode settings-op-save-profile)
                    (profile-save-request data request-id)
                    (if (= opcode settings-op-commit-profile)
                        (profile-commit-request data request-id))))))
    })
}))

(def profile-hide-icon (fn () {
    (if (!= image-profile 0) {
        (if (= resource-blank 0)
            (setq resource-blank (lv-image-load image-blank-data)))
        (lv-image-set-src image-profile resource-blank)
        (lv-object-align image-profile align-left-mid 55 0)
    })
}))

; Feed a payload through Express' built-in VESC packet decoder. This keeps CAN
; fragmentation, CRC handling, and reply routing in the native VESC stack while
; allowing the package to own autonomous profile synchronization.
(def guardian-send-payload (fn (payload) {
    (var payload-length (buflen payload))
    (if (> payload-length 255)
        nil
        {
            (var frame (array-create (+ payload-length 5)))
            (var checksum (crc16 payload))
            (bufset-u8 frame 0 2)
            (bufset-u8 frame 1 payload-length)
            (looprange i 0 payload-length
                (bufset-u8 frame (+ i 2) (bufget-u8 payload i)))
            (bufset-u8 frame (+ payload-length 2) (shr checksum 8))
            (bufset-u8 frame (+ payload-length 3)
                (bitwise-and checksum 0xFF))
            (bufset-u8 frame (+ payload-length 4) 3)
            (cmds-proc frame)
        })
}))

(def guardian-send-forward (fn (target command) {
    (var command-length (buflen command))
    (var payload (array-create (+ command-length 2)))
    (bufset-u8 payload 0 command-forward-can)
    (bufset-u8 payload 1 target)
    (looprange i 0 command-length
        (bufset-u8 payload (+ i 2) (bufget-u8 command i)))
    (guardian-send-payload payload)
}))

(def guardian-build-set-command (fn (profile) {
    ; COMM_SET_MCCONF_TEMP_SETUP payload: flags followed by eight IEEE f32
    ; values. bufset-f32 uses the same big-endian representation as VESC's
    ; buffer_append_float32_auto.
    (var command (array-create 37))
    (bufset-u8 command 0 command-set-mcconf-temp-setup)
    (bufset-u8 command 1 0) ; store = false
    (bufset-u8 command 2 0) ; forward_can = false
    (bufset-u8 command 3 1) ; acknowledgement required
    (bufset-u8 command 4 0) ; divide_by_controllers = false
    (bufset-f32 command 5 (/ (profile-value profile 0) 100.0))
    (bufset-f32 command 9 (/ (profile-value profile 1) 100.0))
    (bufset-f32 command 13 (- (/ (profile-value profile 2) 36.0)))
    (bufset-f32 command 17 (/ (profile-value profile 3) 36.0))
    (bufset-f32 command 21 (/ (profile-value profile 4) 100.0))
    (bufset-f32 command 25 (/ (profile-value profile 5) 100.0))
    (bufset-f32 command 29 (- (profile-value profile 6)))
    (bufset-f32 command 33 (profile-value profile 7))
    command
}))

(def guardian-build-get-command (fn () {
    (var command (array-create 1))
    (bufset-u8 command 0 command-get-mcconf-temp)
    command
}))

(def guardian-handle-command-response (fn (frame) {
    (if (and (!= guardian-command-pending guardian-command-none)
             (>= (buflen frame) 6)
             (= (bufget-u8 frame 0) 2)) {
        (var payload-length (bufget-u8 frame 1))
        (if (and (>= payload-length 1)
                 (>= (buflen frame) (+ payload-length 5))
                 (= (bufget-u8 frame (+ payload-length 4)) 3)) {
            (var payload (array-create payload-length))
            (looprange i 0 payload-length
                (bufset-u8 payload i (bufget-u8 frame (+ i 2))))
            (var checksum (crc16 payload))
            (var received-checksum
                (+ (shl (bufget-u8 frame (+ payload-length 2)) 8)
                   (bufget-u8 frame (+ payload-length 3))))
            (if (= checksum received-checksum) {
                (var response-command (bufget-u8 payload 0))
                (if (and (= guardian-command-pending guardian-command-set)
                         (= response-command command-set-mcconf-temp-setup)) {
                    (setq guardian-response-ready true)
                })
                (if (and (= guardian-command-pending guardian-command-get)
                         (= response-command command-get-mcconf-temp)
                         (>= payload-length 33)) {
                    (setq guardian-response-fingerprint (crc32 payload 0))
                    (setq guardian-response-ready true)
                })
            })
        })
    })
}))

(def guardian-wait-response (fn () {
    (var started (systime))
    (loopwhile (and (not guardian-response-ready)
                    (< (secs-since started)
                       guardian-response-timeout-seconds))
        (sleep 0.02))
    guardian-response-ready
}))

(def guardian-set-profile (fn (target profile) {
    (setq guardian-command-pending guardian-command-set)
    (setq guardian-response-ready nil)
    (setq guardian-response-fingerprint -1)
    (guardian-send-forward target (guardian-build-set-command profile))
    (var success (guardian-wait-response))
    (setq guardian-command-pending guardian-command-none)
    success
}))

(def guardian-read-fingerprint (fn (target) {
    (setq guardian-command-pending guardian-command-get)
    (setq guardian-response-ready nil)
    (setq guardian-response-fingerprint -1)
    (guardian-send-forward target (guardian-build-get-command))
    (var success (guardian-wait-response))
    (var fingerprint (if success guardian-response-fingerprint -1))
    (setq guardian-command-pending guardian-command-none)
    fingerprint
}))

(def guardian-apply-profile (fn (profile request-id) {
    (var target controller-id)
    (setq guardian-busy true)
    (setq guardian-last-attempt-tick (systime))
    (var accepted (and (msg-fresh target 1)
                       (guardian-set-profile target profile)))
    (if accepted {
        (setq selected-profile profile)
        (setq profile-enabled 1)
        (setq applied-profile profile)
        (setq guardian-synced-controller target)
        (setq guardian-fingerprint -1)
        (setq guardian-last-verify-tick (systime))
        (settings-write-if-changed profile-eeprom-selected selected-profile)
        (settings-write-if-changed profile-eeprom-enabled profile-enabled)
        ; An icon allocation failure is contained and cannot stop either the
        ; dashboard or the profile guardian.
        (trap (profile-show-icon applied-profile))
        (if (> request-id 0)
            (profile-send-response 0x85 request-id 0))
        (var fingerprint (guardian-read-fingerprint target))
        (if (>= fingerprint 0)
            (setq guardian-fingerprint fingerprint))
    } {
        (setq applied-profile -1)
        (setq guardian-synced-controller -1)
        (setq guardian-fingerprint -1)
        (trap (profile-hide-icon))
        (if (> request-id 0)
            (profile-send-response 0x85 request-id 3))
    })
    (setq guardian-busy nil)
}))

(def guardian-loop (fn () {
    (loopwhile t {
        (var live (msg-fresh controller-id 1))
        (if (not live) {
            (if (>= guardian-synced-controller 0) {
                (setq applied-profile -1)
                (setq guardian-synced-controller -1)
                (setq guardian-fingerprint -1)
                (trap (profile-hide-icon))
            })
        } {
            (if (>= guardian-request-profile 0) {
                (var requested-profile guardian-request-profile)
                (var requested-id guardian-request-id)
                (setq guardian-request-profile -1)
                (setq guardian-request-id 0)
                (guardian-apply-profile requested-profile requested-id)
            })
            (if (and (= profile-enabled 1)
                     (not guardian-busy)
                     (< guardian-request-profile 0)) {
                (if (!= guardian-synced-controller controller-id) {
                    (if (> (secs-since guardian-last-attempt-tick)
                           guardian-retry-seconds)
                        (guardian-apply-profile selected-profile 0))
                } {
                    (if (> (secs-since guardian-last-verify-tick)
                           guardian-verify-seconds) {
                        (setq guardian-last-verify-tick (systime))
                        (var fingerprint
                            (guardian-read-fingerprint controller-id))
                        (if (>= fingerprint 0) {
                            (if (< guardian-fingerprint 0)
                                (setq guardian-fingerprint fingerprint)
                                (if (!= fingerprint guardian-fingerprint)
                                    (guardian-apply-profile selected-profile 0)))
                        })
                    })
                })
            })
        })
        (sleep 0.25)
    })
}))

(def profile-runtime-start (fn () {
    ; Run after the physical dashboard is already loaded. Any profile storage
    ; or command-interface error is isolated from the visible display runtime.
    (sleep 0.10)
    (var load-result (trap (profile-load)))
    (if (eq (car load-result) 'exit-ok) {
        (var command-result (trap (cmds-start-stop true)))
        (if (eq (car command-result) 'exit-ok) {
            (event-enable 'event-cmds-data-tx)
            (setq profile-ready true)
            (guardian-loop)
        } (print "LVGL profile command bridge failed"))
    } (print "LVGL profile storage initialization failed"))
}))

(def settings-event-handler (fn () {
    (loopwhile t
        (recv
            ((event-data-rx . (? data)) {
                (var settings-result (trap (settings-receive data)))
                (if (not (eq (car settings-result) 'exit-ok))
                    (print "LVGL settings request failed"))
            })
            ((event-cmds-data-tx . (? data)) {
                (var guardian-result
                    (trap (guardian-handle-command-response data)))
                (if (not (eq (car guardian-result) 'exit-ok))
                    (print "LVGL guardian response failed"))
            })
            (_ nil)))
}))

(def live-update (fn () {
    ; Refresh discovery once per second. can-list-devs is non-blocking and
    ; contains devices from received status frames; can-scan is deliberately
    ; not used in the display loop because it actively pings every CAN ID.
    (if (= (mod live-tick 10) 0)
        (select-controller))

    (var status-1-fresh (msg-fresh controller-id 1))
    (var status-4-fresh (msg-fresh controller-id 4))
    (var status-5-fresh (msg-fresh controller-id 5))

    ; Speed: status 1 electrical RPM, converted with the calibration above.
    (if status-1-fresh {
        (var speed-sample (clamp (erpm-to-kmh (canget-rpm controller-id)) 0.0 999.0))
        (if speed-filter-valid
            (setq speed-filter (+ speed-filter
                (* telemetry-filter-alpha (- speed-sample speed-filter))))
            {
                (setq speed-filter speed-sample)
                (setq speed-filter-valid true)
            })
        (var displayed-speed (* speed-filter
            (if (= speed-unit 0) 1.0 0.621371)))
        (lv-label-text label-speed
            (str-from-n (round-int displayed-speed) "%02d"))
    } {
        (setq speed-filter-valid nil)
        (lv-label-text label-speed "--")
    })

    ; Battery: prefer fresh VESC BMS SOC, otherwise scale status 5 input
    ; voltage using the package calibration.
    (var bms-age (get-bms-val 'bms-msg-age))
    (var bms-soc (get-bms-val 'bms-soc))
    (var bms-fresh (and (number? bms-age) (number? bms-soc)
        (< bms-age telemetry-stale-seconds) (>= bms-soc 0.0) (<= bms-soc 1.0)))
    (var battery-valid (or bms-fresh status-5-fresh))
    (if battery-valid {
        (var battery-sample (if bms-fresh
            (* bms-soc 100.0)
            (vin-to-percent (canget-vin controller-id))))
        (if battery-filter-valid
            (setq battery-filter (+ battery-filter
                (* telemetry-filter-alpha (- battery-sample battery-filter))))
            {
                (setq battery-filter battery-sample)
                (setq battery-filter-valid true)
            })
        (var battery-value (clamp (round-int battery-filter) 0 100))
        (lv-label-text label-battery (str-from-n battery-value "%d%%"))
        (lv-arc-value arc-battery battery-value)
    } {
        (setq battery-filter-valid nil)
        (lv-label-text label-battery "--%")
        (lv-arc-value arc-battery 0)
    })

    ; Controller and motor temperatures come from CAN status 4. A missing
    ; motor sensor is normally reported near -100 C, so reject that sentinel.
    (var controller-temp (if status-4-fresh (canget-temp-fet controller-id) -1000.0))
    (var motor-temp (if status-4-fresh (canget-temp-motor controller-id) -1000.0))
    (var controller-valid (and status-4-fresh (> controller-temp -40.0) (< controller-temp 250.0)))
    (var motor-valid (and status-4-fresh (> motor-temp -40.0) (< motor-temp 250.0)))

    (if controller-valid {
        (if controller-filter-valid
            (setq controller-filter (+ controller-filter
                (* telemetry-filter-alpha (- controller-temp controller-filter))))
            {
                (setq controller-filter controller-temp)
                (setq controller-filter-valid true)
            })
        (lv-label-text label-controller (str-from-n controller-filter "%.1fc"))
        (lv-arc-value arc-controller
            (clamp (round-int controller-filter) 0 controller-temp-arc-max))
    } {
        (setq controller-filter-valid nil)
        (lv-label-text label-controller "--.-c")
        (lv-arc-value arc-controller 0)
    })

    (if motor-valid {
        (if motor-filter-valid
            (setq motor-filter (+ motor-filter
                (* telemetry-filter-alpha (- motor-temp motor-filter))))
            {
                (setq motor-filter motor-temp)
                (setq motor-filter-valid true)
            })
        (lv-label-text label-motor (str-from-n motor-filter "%.1fc"))
        (lv-arc-value arc-motor
            (clamp (round-int motor-filter) 0 motor-temp-arc-max))
    } {
        (setq motor-filter-valid nil)
        (lv-label-text label-motor "--.-c")
        (lv-arc-value arc-motor 0)
    })

    ; Power mode 0 is signed battery input watts. Mode 1 is the same signed
    ; status-4 battery/input current directly in amps.
    (var power-valid (and status-4-fresh
        (or (= power-unit 1) status-5-fresh)))
    (if power-valid {
        (var input-current (canget-current-in controller-id))
        (var power-sample (if (= power-unit 0)
            (* (canget-vin controller-id) input-current)
            input-current))
        (if power-filter-valid
            (setq power-filter (+ power-filter
                (* telemetry-filter-alpha (- power-sample power-filter))))
            {
                (setq power-filter power-sample)
                (setq power-filter-valid true)
            })
        (var power-value (clamp (round-int power-filter)
            (if (= power-unit 0) -9999 -999)
            (if (= power-unit 0) 9999 999)))
        (if (and (= power-unit 0) (<= (abs power-value) 5))
            (setq power-value 0))
        (lv-label-text label-power (str-from-n power-value "%d"))
    } {
        (setq power-filter-valid nil)
        (lv-label-text label-power "----")
    })

    ; Diagnostic heartbeat for VESC Tool package/runtime verification.
    (if (= (mod live-tick 10) 0)
        (send-data (str-merge "lvgl-live id="
            (str-from-n controller-id "%d") " s1="
            (str-from-n (if status-1-fresh 1 0) "%d") " s4="
            (str-from-n (if status-4-fresh 1 0) "%d") " s5="
            (str-from-n (if status-5-fresh 1 0) "%d") " profile="
            (str-from-n selected-profile "%d") " sync="
            (str-from-n
                (if (and (= profile-enabled 1)
                         (= guardian-synced-controller controller-id)) 1 0)
                "%d") "\n")))
    (setq live-tick (+ live-tick 1))
}))

(def main (fn () {
    (if (not (lv-ready)) {
        (print "LVGL package bridge is not available")
        (exit-error 'lvgl-not-ready)
    })

    (lv-reset)
    (def font-speed (lv-font-load font-speed-data))
    (def font-large (lv-font-load font-large-data))
    (def font-medium (lv-font-load font-medium-data))
    (def font-small (lv-font-load font-small-data))
    (def resource-battery (lv-image-load image-battery-data))
    (def resource-controller (lv-image-load image-controller-data))
    (def resource-motor (lv-image-load image-motor-data))
    (def resource-eco 0)
    (def resource-race 0)
    (def resource-blank 0)

    (def screen (lv-screen-create))
    (lv-object-style-bg screen color-black 255 0)

    ; Three moving perimeter gauges for temperature and battery state.
    (def arc-controller (make-arc screen 452 controller-temp-arc-max 10 70 arc-mode-normal 5))
    (def arc-motor (make-arc screen 456 motor-temp-arc-max 290 350 arc-mode-reverse 5))
    (def arc-battery (make-arc screen 456 100 110 250 arc-mode-normal 5))

    (make-image screen resource-controller align-center 152 110 color-white)
    (make-image screen resource-motor align-center 163 -100 color-white)
    (make-image screen resource-battery align-center -189 -2 color-white)

    (def label-controller (make-label screen font-small "--.-c" 84 size-content
        align-right-mid -5 15 color-white text-align-right))
    (def label-motor (make-label screen font-small "--.-c" 84 size-content
        align-right-mid -5 -15 color-white text-align-right))
    (def label-battery (make-label screen font-medium "--%" 120 size-content
        align-top-mid -8 10 color-white text-align-right))

    ; Central speed and power card.
    (def speed-card (lv-object-create screen))
    (lv-object-remove-style-all speed-card)
    (lv-object-set-size speed-card 260 260)
    (lv-object-align speed-card align-center 0 0)
    (lv-object-style-bg speed-card color-black 0 220)
    (lv-object-style-border speed-card color-gold 255 2)

    (def label-speed (make-label speed-card font-speed "--" 260 size-content
        align-top-mid 0 40 color-white text-align-center))
    (def label-speed-unit (make-label speed-card font-medium "kmh" 100 size-content
        align-center 0 10 color-muted text-align-center))
    (def label-power (make-label speed-card font-large "----" 220 size-content
        align-bottom-mid 0 -55 color-white text-align-center))
    (def label-power-unit (make-label speed-card font-medium "watts" 140 size-content
        align-bottom-mid 0 -30 color-muted text-align-center))

    (def controller-id -1)
    (def live-tick 0)
    (def speed-filter 0.0)
    (def battery-filter 0.0)
    (def controller-filter 0.0)
    (def motor-filter 0.0)
    (def power-filter 0.0)
    (def speed-filter-valid nil)
    (def battery-filter-valid nil)
    (def controller-filter-valid nil)
    (def motor-filter-valid nil)
    (def power-filter-valid nil)
    (def profile-values (make-profile-default-values))
    (def selected-profile 0)
    (def profile-enabled 0)
    (def profile-ready nil)
    (def applied-profile -1)
    (def image-profile 0)
    (def guardian-busy nil)
    (def guardian-request-profile -1)
    (def guardian-request-id 0)
    (def guardian-synced-controller -1)
    (def guardian-fingerprint -1)
    (def guardian-last-attempt-tick 0)
    (def guardian-last-verify-tick 0)
    (def guardian-command-pending guardian-command-none)
    (def guardian-response-ready nil)
    (def guardian-response-fingerprint -1)
    (def display-brightness default-brightness)
    (def speed-unit default-speed-unit)
    (def power-unit default-power-unit)

    ; Make the package screen visible before reading persistent settings or
    ; starting optional subsystems. A storage, brightness, profile, command,
    ; or telemetry error must never leave the native Hello LVGL screen active.
    (lv-screen-load screen)
    (var settings-load-result (trap (settings-load)))
    (if (not (eq (car settings-load-result) 'exit-ok))
        (print "LVGL settings storage initialization failed"))
    (var settings-apply-result (trap (settings-apply)))
    (if (not (eq (car settings-apply-result) 'exit-ok))
        (print "LVGL settings application failed"))

    (event-register-handler (spawn settings-event-handler))
    (event-enable 'event-data-rx)
    (spawn profile-runtime-start)
    (loopwhile-thd ("LVGL-live" 320) t {
        (var live-result (trap (live-update)))
        (if (eq (car live-result) 'exit-ok)
            (sleep 0.10)
            {
                (print "LVGL telemetry update failed")
                (sleep 1.0)
            })
    })
}))

@const-end

(image-save)
(main)
