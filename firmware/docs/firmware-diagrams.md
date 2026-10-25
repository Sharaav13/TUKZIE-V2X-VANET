# TVV Node Firmware: UML and Flow Diagrams

Diagrams of the node firmware in `src/` and `include/` (ESP32-S3, Arduino + FreeRTOS). They are written in Mermaid, which GitHub and VS Code (with a Mermaid extension) render.

1. [UML class diagram](#1-uml-class-diagram): structs, modules, global instances and libraries
2. [UML sequence diagram](#2-uml-sequence-diagram-dual-core-runtime): how the two cores, the radio and the host interact
3. [UML state diagram](#3-uml-state-diagram-vanet-mac-csmaca): VANET MAC (CSMA/CA)
4. [Flow: boot](#4-flow-boot-setup-and-loop): `setup()` and `loop()`
5. [Flow: sensing task](#5-flow-sensing-task-core-0): `v2sLoop()` on core 0
6. [Flow: network task](#6-flow-network-task-core-1): `v2vLoop()` on core 1
7. [Flow: speed fusion](#7-flow-speed-fusion-speedfilterimuupdate): `SpeedFilter::imuUpdate()`
8. [Flow: road awareness and messages](#8-flow-road-awareness-and-message-priority): `checkAccident()`, `checkTerrain()`, `checkBattery()`, `sortMsg()`
9. [Flow: overview](#9-flow-overview): simplified view of the whole firmware

---

## 1. UML class diagram

Every struct has one global instance (`gps`, `imu`, `speedFilter`, `vanet`, `host`, `storage`, `oled`, `lights`). `myself : Data` is the node's own state, shared between the cores under `myselfMutex`. The non-struct source files are shown as `<<module>>` classes.

```mermaid
classDiagram
    direction TB

    %% ---------------- Core data (system.h / system.cpp) ----------------
    class Data {
        <<struct>>
        +byte destId
        +byte srcId
        +uint16_t seq
        +uint8_t ttl
        +int8_t srcRssi
        +float pktRssi
        +float snr
        +float freqErr
        +int64_t utc
        +uint32_t airUs
        +uint8_t sync
        +byte id
        +float lat
        +float lon
        +float alt
        +byte sats
        +float hdop
        +float speed
        +float course
        +uint8_t tasks
        +uint8_t status
        +uint8_t message
        +byte battery
        +uint32_t date
        +uint32_t time
        +toStr() String
    }

    class Errors {
        <<struct>>
        +bool check
        +String msg
        +SemaphoreHandle_t mutex
    }

    class SystemModule {
        <<module system.cpp>>
        +Data myself$
        +Errors errors$
        +reportError(text) void$
    }

    %% ---------------- VANET (vanet.h / vanet.cpp) ----------------
    class Packet {
        <<packed struct, 25 bytes>>
        +uint8_t dest
        +uint8_t src
        +uint16_t seq
        +uint8_t ttl
        +uint8_t message
        +uint32_t date
        +uint32_t time
        +int32_t lat
        +int32_t lon
        +uint8_t status
        +uint8_t tasks
        +uint8_t battery
        +pack(node) void
        +unpack(node) void
    }

    class Cluster {
        <<struct>>
        +Data peerTable[MAX_PEERS = 3]
        +uint8_t count
        +uint32_t lastHeard[MAX_PEERS]
        +uint32_t lastCheck
        +find(id) int
        +update(node) bool
        +removeStale() uint8_t
        +averageSnr() float
    }

    class Vanet {
        <<struct>>
        +SPIClass spi = HSPI
        +volatile bool rxFlag
        +bool rxValid
        +uint8_t peerMsg
        +uint8_t peerMsgId
        +uint8_t lastMsg
        +uint16_t txSeq
        +int8_t txRssi
        +uint32_t lastTx
        +uint32_t backoffStart
        +uint32_t backoffMs
        +Data rxBuffer
        +Cluster cluster
        +init() bool
        +rx() bool
        +readPacket(size) void
        +queueForward(pkt) void
        +forwardDue() bool
        +forward(node) bool
        +clearBuffer() void
        +startBackoff() void
        +backoffActive() bool
        +receiving() bool
        +beaconDue() bool
        +tx(node) bool
    }

    class VanetCallbacks {
        <<module vanet.cpp>>
        +loraDio0ISR() void$
        +onLoraReceive(size) void$
    }

    %% ---------------- Sensing (sensing.h / sensing.cpp) ----------------
    class UtcClock {
        <<struct>>
        +volatile int64_t ppsUs
        +int64_t anchorUs
        +int64_t anchorUtc
        +double rate
        +pair(utcSecond) void
        +toUtc(localUs, sync) int64_t
    }

    class GPS {
        <<struct>>
        +HardwareSerial uart = UART1
        +TinyGPSPlus tinyGps
        +int32_t lastUtcSecond
        +bool detected
        +bool fix
        +float lastSpeed
        +uint32_t lastSample
        +uint32_t lastByte
        +init() bool
        +get() void
        +status() void
        +speedVariance() float
        +toStr(out, cap) size_t
    }

    class IMU {
        <<struct>>
        +LSM6 lsm6
        +uint16_t fails
        +float accBias[3]
        +float gyroBias[3]
        +float acc[3]
        +float gyro[3]
        +uint32_t lastSample
        +init() bool
        +calibrate() bool
        +get(reading, axis, calibrated, scaled) float
        +getAll() bool
        +toStr(out, cap) size_t
    }

    class KalmanFilter {
        <<struct>>
        +float speed
        +float accelBias
        +float P00
        +float P01
        +float P10
        +float P11
        +reset(initialSpeed) void
        +predict(acceleration, dt) void
        +correct(measuredSpeed, measurementVariance, allowHardReset) void
    }

    class SpeedFilter {
        <<struct>>
        +KalmanFilter kalman
        +bool initialized
        +bool stationary
        +uint32_t stationaryStart
        +float roll
        +float pitch
        +float fwdAccelFiltered
        +uint32_t lastGpsCorrection
        +uint32_t lastZupt
        +uint32_t lastFusion
        +init() void
        +gpsUpdate(gpsSpeed) void
        +stationaryUpdate(accMagnitude, gyroMagnitude) void
        +imuUpdate(dt) void
        +update() void
        +getSpeed() float
    }

    %% ---------------- Interpret (interpret.h / interpret.cpp) ----------------
    class Interpret {
        <<module interpret.cpp>>
        +const char* codedMessages[MSG_COUNT]$
        +const uint8_t msgColours[MSG_COUNT]$
        +const char* codedStatus[STATUS_COUNT]$
        +SemaphoreHandle_t msgFlagsMutex$
        -bool msgFlags[MSG_COUNT]$
        +checkAccident() void$
        +checkTerrain() void$
        +checkBattery() void$
        +setMsgFlag(code) void$
        +sortMsg() uint8_t$
    }

    %% ---------------- Host (host.h / host.cpp) ----------------
    class Host {
        <<struct>>
        +HardwareSerial uart = UART2
        +uint32_t lastTx
        +uint32_t lastRx
        +char rxBuffer[MSG_SIZE]
        +uint8_t rxLen
        +bool inPacket
        +init() bool
        +tx(me, table, size) void
        +rx() bool
        +present() bool
    }

    %% ---------------- Storage (storage.h / storage.cpp) ----------------
    class Storage {
        <<struct>>
        +SPIClass spi = FSPI
        +bool ok
        +uint32_t lastWrite
        +uint32_t lastRead
        +uint32_t lastNodeLog
        +uint32_t lastSenseLog
        +init() bool
        +deleteLogs() void
        +configNode() String
        +read(file) String
        +write(content, file) void
        +formatSensing(node) String$
        +formatVanet(event, node) String$
        +formatMyself(node) String$
    }

    %% ---------------- UI (ui.h / ui.cpp) ----------------
    class OLED {
        <<struct>>
        +Adafruit_SSD1306 dev
        +bool ok
        +uint32_t lastShow
        +init() bool
        +centerText(text) void
        +disp(text, size, center, append) void
        +refresh() void
        +showNode(me) void
    }

    class SystemLights {
        <<struct>>
        +Adafruit_NeoPixel dev
        +uint8_t colour
        +bool flashing
        +uint32_t flashStart
        +init() bool
        +toggle(colour) void
        +flash(colour) void
        +update() void
    }

    %% ---------------- Main and debugging ----------------
    class Main {
        <<module main.cpp>>
        +bool debugMode$
        +bool hostReady$
        +bool sdReady$
        +bool gpsReady$
        +bool imuReady$
        +bool vanetReady$
        +TaskHandle_t v2sTask$
        +TaskHandle_t v2vTask$
        +SemaphoreHandle_t myselfMutex$
        +setup() void$
        +loop() void$
        +v2sLoop(param) void$
        +v2vLoop(param) void$
        +copyMyself() Data$
        +initStatus(name, ready) String$
        +showInit(name, ready, append, holdMs) void$
    }

    class Debugging {
        <<module debugging.cpp>>
        +debug(peripheral) void$
        +uiDebug() void$
        +storageDebug() void$
        +gpsDebug() void$
        +imuDebug() void$
        +hostDebug() void$
        +loraDebug(select) void$
    }

    %% ---------------- External libraries ----------------
    class LoRaClass {
        <<library, patched lib/LoRa>>
    }
    class TinyGPSPlus {
        <<library>>
    }
    class LSM6 {
        <<library>>
    }
    class Adafruit_SSD1306 {
        <<library>>
    }
    class Adafruit_NeoPixel {
        <<library>>
    }
    class SD {
        <<library>>
    }

    %% ---------------- Relationships ----------------
    SystemModule *-- "1 myself" Data
    SystemModule *-- "1" Errors

    Vanet *-- "1" Cluster
    Vanet *-- "1 rxBuffer" Data
    Cluster *-- "0..3 peers" Data
    Vanet ..> Packet : pack / unpack
    Packet ..> Data : converts
    Vanet ..> LoRaClass : SPI radio
    VanetCallbacks ..> Vanet : sets rxFlag / readPacket
    LoRaClass ..> VanetCallbacks : onReceive callback

    SpeedFilter *-- "1" KalmanFilter
    SpeedFilter ..> IMU : getAll, biases
    SpeedFilter ..> GPS : fix, lastSpeed
    GPS ..> SpeedFilter : gpsUpdate
    IMU ..> SpeedFilter : init after calibrate
    GPS *-- TinyGPSPlus
    IMU *-- LSM6
    GPS ..> Data : writes myself

    Interpret ..> IMU : reads acc, gyro
    Interpret ..> Data : reads myself.battery

    Host ..> Data : writes status, tasks, battery, destId, message
    Storage ..> Data : writes id
    Storage ..> SD

    OLED *-- Adafruit_SSD1306
    SystemLights *-- Adafruit_NeoPixel
    OLED ..> Vanet : cluster, peerMsg
    OLED ..> Host : present
    OLED ..> Interpret : codedStatus, codedMessages

    Main ..> Debugging : debugMode only
    note for Main "Uses every module: setup() inits each one,
v2sLoop() on core 0 and v2vLoop() on core 1 drive them"
```

---

## 2. UML sequence diagram: dual-core runtime

After `setup()`, the work is split across two FreeRTOS tasks. Core 0 writes `myself`, and core 1 reads it through `copyMyself()`. Both lock `myselfMutex`. Message codes pass between the cores through `msgFlags`, which `msgFlagsMutex` protects.

```mermaid
sequenceDiagram
    autonumber
    participant HW as Host system (UART2)
    participant S as v2sLoop (core 0)
    participant M as myself + myselfMutex
    participant F as msgFlags + msgFlagsMutex
    participant N as v2vLoop (core 1)
    participant V as Vanet
    participant L as LoRa lib + SX127x
    participant ISR as loraDio0ISR

    loop every ~1 ms on core 0
        S->>S: speedFilter.update() (IMU predict @100 Hz)
        S->>M: take mutex
        S->>M: gps.get() writes date, time, lat, lon, alt, course, sats, hdop
        S->>M: myself.speed = speedFilter.getSpeed()
        S->>F: checkAccident() / checkTerrain() setMsgFlag(1, 3, 4)
        HW-->>S: "$status,tasks,battery,dest,message\n"
        S->>M: host.rx() writes status, tasks, battery, destId (message only if 4)
        S->>F: checkBattery() setMsgFlag(2) if host present
        S->>M: give mutex
    end

    Note over L,ISR: Peer packet arrives (RxDone on DIO0)
    L-)ISR: DIO0 rising edge
    ISR->>V: rxFlag = true (no SPI in ISR)

    loop every ~1 ms on core 1
        N->>V: rx()
        alt rxFlag set
            V->>V: startBackoff() (0-300 ms random)
            V->>L: handleDio0Rise()
            L->>V: onLoraReceive(size) to readPacket(size)
            V->>V: repeat of a (src, seq) read in the last 30 s: dropped
            V->>V: size == 25 and dest is me or 0: unpack, pktRssi/snr, cluster.update (direct only, ttl 2), peerMsg, log RX
            V->>V: size == 25 and dest is another node (not me, not 0): queueForward (ttl - 1, not at 0)
        end
        N->>V: forwardDue()?
        alt packet queued, back-off over and channel free
            N->>V: forward(fwd)
            V->>L: beginPacket, write the queued 25 bytes as received, endPacket, receive()
            N->>N: log FW
        end
        N->>V: beaconDue()?
        alt interval elapsed, back-off over and channel free
            N->>M: copyMyself()
            N->>F: host message (4) if set, else sortMsg() (highest priority code, clears flag)
            N->>V: tx(me)
            V->>L: seq + 1, ttl = 2, srcRssi = LoRa.rssi(), beginPacket, write 25 bytes, endPacket, receive()
            N->>N: lights.flash(green, blue for the host message), log TX, clear host message once sent
        end
        N->>V: cluster.removeStale() every 1 s
        N->>M: copyMyself()
        N->>HW: host.tx(): "$id,myself\n" then "#id,peer\n" per peer (every 1 s)
        N->>N: oled.showNode() (every 1 s or on Rx)
        N->>N: lights.update()
    end
```

---

## 3. UML state diagram: VANET MAC (CSMA/CA)

The radio stays in continuous Rx. `rx()` and `beaconDue()` run on every network loop pass, each on its own.

```mermaid
stateDiagram-v2
    [*] --> Listening : vanet.init() LoRa.receive(), random first beacon offset

    Listening --> PacketReady : DIO0 RxDone ISR sets rxFlag
    PacketReady --> BackingOff : rx() startBackoff() random 0-300 ms, handleDio0Rise() readPacket()

    BackingOff --> BackingOff : another packet received, back-off restarts
    BackingOff --> Listening : back-off elapsed

    Listening --> CarrierSense : BEACON_INTVL (15 s) since lastTx
    CarrierSense --> Listening : receiving() RegModemStat & 0x0B, channel busy
    CarrierSense --> Transmitting : channel free
    Transmitting --> Listening : endPacket() (~72 ms), LoRa.receive(), lights.flash()
    Transmitting --> Listening : beginPacket() false, radio busy, message re-flagged

    note right of BackingOff
        beaconDue() returns false while
        the back-off is active
    end note
```

---

## 4. Flow: boot (`setup()` and `loop()`)

```mermaid
flowchart TD
    A([Power on / reset]) --> B["Serial.begin(115200)"]
    B --> C["Create errors.mutex and msgFlagsMutex"]
    C --> D["lights.init() RGB LED, white until the first init result"]
    D --> E["hostReady = host.init() UART2 + RX pull-up"]
    E --> F["oled.init() I2C SSD1306 @0x3C"]
    F --> G["Splash: 'TUKZIE V2X' 2 s, 'Booting...' 2 s"]
    G --> H["showInit Host"]
    H --> I["sdReady = storage.init() FSPI + SD.begin, deleteLogs() (config.txt kept)"]
    I --> I2["showInit SD"]
    I2 --> J["gpsReady = gps.init() UART1 9600"]
    J --> J2["showInit GPS"]
    J2 --> K["imuReady = imu.init()"]
    K --> K1["Wire1.begin, lsm6.init, enableDefault"]
    K1 --> K2["calibrate(): average 1000 samples into accBias / gyroBias"]
    K2 --> K3["speedFilter.init(): roll/pitch from gravity, kalman.reset(0)"]
    K3 --> K4["showInit IMU"]
    K4 --> L["vanetReady = vanet.init()"]
    L --> L1["HSPI + LoRa.begin 433.2125 MHz, SF7, BW125k, CR4/5, sync 0x12, 10 dBm, CRC"]
    L1 --> L2["Random first beacon offset, replace DIO0 ISR with loraDio0ISR, LoRa.receive()"]
    L2 --> L3["showInit LoRa"]
    L3 --> M["storage.configNode(): /config.txt node ID into myself"]
    M --> N{"errors.check?"}
    N -- yes --> N1["reportError('Setup: errors occurred during boot')"]
    N1 --> O
    N -- no --> O["OLED 'Finished boot.' 3 s"]
    O --> P{"errors.check?"}
    P -- yes --> P1["Show errors.msg, RGB red 3 s"]
    P1 --> Q
    P -- no --> Q{"debugMode?"}
    Q -- yes --> DBG["return: loop() runs debug(DBG_x) forever"]
    Q -- no --> R["Create myselfMutex"]
    R --> S{"Any mutex NULL?"}
    S -- yes --> S1["reportError, show errors, RGB red, return (halt)"]
    S -- no --> T["xTaskCreatePinnedToCore v2sLoop on core 0"]
    T --> U["xTaskCreatePinnedToCore v2vLoop on core 1"]
    U --> V["loop(): vTaskDelete(NULL), Arduino loop task ends"]

    subgraph showInit["showInit(name, ready)"]
        direction LR
        SI1["oled.disp('name init pass/failed')"] --> SI2["RGB green if pass, red if fail"] --> SI3["delay(holdMs)"] --> SI4["RGB off"]
    end
```

---

## 5. Flow: sensing task (core 0)

```mermaid
flowchart TD
    A([v2sLoop start]) --> B{"imuReady?"}
    B -- yes --> C["speedFilter.update()"]
    C --> C1{"≥ 10 ms since lastFusion?"}
    C1 -- yes --> C2{"1 ms < dt < 100 ms?"}
    C2 -- yes --> C3["imuUpdate(dt): Kalman PREDICT (see section 7)"]
    C2 -- no --> D
    C1 -- no --> D
    C3 --> D
    B -- no --> D["Take myselfMutex"]

    D --> E["gps.get()"]
    E --> E1["Feed UART bytes to TinyGPSPlus, update lastByte"]
    E1 --> E2["status(): detected (data < 3 s), fix (valid, < 2 s old, speed valid)"]
    E2 --> E3["Date/time UTC to SAST (+2 h, day/month/year rollover): DDMMYY, HHMMSSCC"]
    E3 --> E4["lat/lon if updated, alt, course, sats, hdop into myself"]
    E4 --> E5{"New GPS speed?"}
    E5 -- yes --> E6["speedFilter.gpsUpdate(): first fix seeds kalman, then CORRECT"]
    E5 -- no --> F
    E6 --> F["myself.speed = speedFilter.getSpeed() (kmph)"]

    F --> G{"imuReady?"}
    G -- yes --> G1["checkAccident()"] --> G2["checkTerrain()"] --> H
    G -- no --> H{"hostReady?"}
    H -- yes --> H1["host.rx(): parse '$status,tasks,battery,dest,message\n' into myself (message only if 4)"] --> I
    H -- no --> I{"host.present()? (valid packet < 10 s ago)"}
    I -- yes --> I1["checkBattery()"] --> J
    I -- no --> J["Give myselfMutex"]
    J --> K["vTaskDelay(1), lets the idle task feed the watchdog"]
    K --> B
```

---

## 6. Flow: network task (core 1)

```mermaid
flowchart TD
    A([v2vLoop start]) --> B{"vanetReady?"}
    B -- no --> H

    B -- yes --> C["received = vanet.rx()"]
    C --> C1{"rxFlag?"}
    C1 -- no --> D
    C1 -- yes --> C2["rxFlag = false, startBackoff() random 0-300 ms"]
    C2 --> C3["LoRa.handleDio0Rise() then onLoraReceive(size) then readPacket(size)"]
    C3 --> C4{"size == 25 bytes?"}
    C4 -- no --> D
    C4 -- yes --> CD{"(src, seq) already read<br/>in the last 30 s?"}
    CD -- yes --> CD1["Drop the repeat"] --> D
    CD -- no --> CF{"dest is another node<br/>(not me, not 0)?"}
    CF -- yes --> CF1{"ttl - 1 > 0 AND<br/>queue not full?"}
    CF1 -- yes --> CF2["queueForward: ttl - 1, queued as received"] --> D
    CF1 -- no --> D
    CF -- no --> C5["Packet.unpack into rxBuffer, add pktRssi, snr, freqErr"]
    C5 --> CT{"ttl == 2?<br/>(heard straight from the sender)"}
    CT -- yes --> C6["cluster.update(rxBuffer): add or refresh peer (max 3)"] --> C7
    CT -- no --> C7["peerMsg / peerMsgId / peerMsgAt = sender's message"]
    C7 --> C8["storage.write(formatVanet('RX', rxBuffer), vanet_log{ID}.txt)"]
    C8 --> F0

    F0{"vanet.forwardDue()?<br/>packet queued AND<br/>back-off over AND<br/>!receiving()"}
    F0 -- yes --> F1["vanet.forward(fwd): send the oldest queued packet as received, receive()"]
    F1 --> F2["storage.write(formatVanet('FW', fwd), vanet_log{ID}.txt)"] --> D
    F0 -- no --> D

    D{"vanet.beaconDue()?<br/>≥ 2 s since lastTx AND<br/>back-off over AND<br/>!receiving()"}
    D -- no --> G
    D -- yes --> E["me = copyMyself(), me.message = 5 if the host set it, else sortMsg()"]
    E --> E1["vanet.tx(me): seq + 1, ttl = 2, srcRssi = LoRa.rssi(), Packet.pack, beginPacket, write 25 B, endPacket, receive()"]
    E1 --> E2{"Sent?"}
    E2 -- yes --> E3["lights.flash(green, or blue for the host message), 500 ms,<br/>log TX, clear the host message"]
    E2 -- no --> E4{"message != 0 AND not the host message?"}
    E4 -- yes --> E5["setMsgFlag(message), retry next beacon"]
    E4 -- no --> G
    E3 --> G
    E5 --> G

    G{"≥ 1 s since cluster.lastCheck?"}
    G -- yes --> G1["cluster.removeStale(): drop peers silent > 10 s"]
    G1 --> G2{"peerMsgId not in the cluster AND<br/>peerMsg older than 10 s?"}
    G2 -- yes --> G3["peerMsg = 0"] --> H
    G2 -- no --> H
    G -- no --> H

    H{"hostReady AND ≥ 1 s since host.lastTx?"}
    H -- yes --> H1["me = copyMyself(), me.message = vanet.lastMsg, me.seq = txSeq, me.srcRssi = txRssi"]
    H1 --> H2["host.tx(): '$id,myself\n' then '#id,peer\n' per cluster peer"]
    H2 --> I
    H -- no --> I

    I{"received OR ≥ 1 s since oled.lastShow?"}
    I -- yes --> I1["oled.showNode(copyMyself()): signal bars (average peer SNR), speed, battery, status, lat/lon, peers, latest peer message"]
    I1 --> J
    I -- no --> J["lights.setBackground(): red if detected(1), else purple if detected(2),<br/>else orange if detected(3) (each held 200 ms), else yellow without a GPS fix, else off; lights.update(): end the flash after 500 ms"]

    J --> K["Every 1 s: log myself_log{ID}.txt and sensor_log{ID}.txt"]
    K --> L["vTaskDelay(1)"]
    L --> B
```

---

## 7. Flow: speed fusion (`SpeedFilter::imuUpdate`)

The state is 2-D: x = [speed (m/s), forward accelerometer bias (m/s²)]. The IMU runs the predict step at 100 Hz. The GPS runs the correct step at about 1 Hz, and zero-velocity updates (ZUPT) also correct while the node is stopped.

```mermaid
flowchart TD
    A([imuUpdate dt]) --> B{"imu.getAll() OK?"}
    B -- no --> B1["imu.fails++, reportError('IMU: connection lost') once at 101 fails"] --> Z([return])
    B -- yes --> C["fails = 0, |acc| and |gyro| magnitudes"]
    C --> D["stationaryUpdate(): GPS fresh < 0.4 m/s AND gyro < 1.5 dps AND |acc-1g| < 0.06 g, held 700 ms"]
    D --> E{"stationary?"}
    E -- yes --> E1["Refine imu.gyroBias (alpha 0.0015)"] --> F
    E -- no --> F["Attitude: gyro Euler rates + gravity roll/pitch, complementary filter (0.997 gyro, less acc trust off 1 g)"]
    F --> G["fwdAccel = ax × g − g × sin(pitch)"]
    G --> H["Low-pass (alpha 0.12), deadband 0.06 m/s², clamp ±4 m/s²"]
    H --> I{"initialized AND GPS correction < 3 s ago AND !stationary AND |gyro| ≤ 30 dps?"}
    I -- yes --> I1["kalman.predict(usedAccel, dt): speed += (a − bias) × dt, P = FPF^T + Q"] --> J
    I -- no --> J{"stationary AND initialized?"}
    J -- no --> Z
    J -- yes --> J1["Learn accelBias towards fwdAccelFiltered (alpha 0.01)"]
    J1 --> K{"≥ 100 ms since lastZupt?"}
    K -- no --> Z
    K -- yes --> K1["ZUPT: kalman.correct(0, 0.0025), snap speed < 0.03 to 0, decay fwdAccelFiltered"] --> Z

    subgraph GPS["gpsUpdate(gpsSpeed) from gps.get()"]
        direction TB
        Q1{"gps.fix?"} -- no --> Q0([ignore])
        Q1 -- yes --> Q2{"initialized?"}
        Q2 -- no --> Q3["kalman.reset(gpsSpeed), initialized = true"]
        Q2 -- yes --> Q4["kalman.correct(gpsSpeed, speedVariance(), hard reset if |innovation| > 2.5)"]
    end
```

---

## 8. Flow: road awareness and message priority

```mermaid
flowchart TD
    subgraph ACC["checkAccident()"]
        direction TB
        A5{"Impact: magnitude of a ≥ 3 g?<br/>(any direction, gravity included)"} -- yes --> A3["setMsgFlag(1) Met with accident"]
        A1{"|ax| ≥ 0.8 g OR |ay| ≥ 0.8 g OR az ≤ -0.8 g?<br/>(on its side / upside down)"}
        A1 -- yes --> A2{"Held ≥ 10 s?"} -- yes --> A3
        A1 -- no --> A4["toppled = false"]
    end

    subgraph TER["checkTerrain()"]
        direction TB
        T1["acc_z = az - accBias z (gravity removed)"] --> T2{"abs(acc_z) ≥ 0.8 g<br/>AND speed ≥ 5 km/h?"}
        T2 -- yes --> T3["setMsgFlag(3) Bad terrain"]
    end

    subgraph BAT["checkBattery() (only while host present)"]
        direction TB
        B1{"myself.battery ≤ 50 %?"} -- yes --> B2["setMsgFlag(2) Low on battery"]
    end

    subgraph PRI["sortMsg() at each beacon (core 1)"]
        direction TB
        P0["Host message (4) set? Sent first, flags left pending"] --> P1["Lock msgFlagsMutex"]
        P1 --> P2["Pick lowest set code (1 accident > 2 battery > 3 bad terrain), clear it"]
        P2 --> P3["Unlock, return code (0 = none)"]
        P3 --> P4["Beacon carries code, RGB flashes:<br/>0 green, 1 red, 2 purple, 3 orange, 4 yellow, 5 blue"]
    end

    A3 --> PRI
    T6 --> PRI
    T7 --> PRI
    B2 --> PRI
```

---

## 9. Flow: overview

A simplified view of the whole firmware: boot, then two tasks running in parallel, one per core. The V2S task (core 0) writes the node's own state (`myself`) and raises message flags; the V2V task (core 1) reads them. Both are protected by mutexes.

```mermaid
flowchart TD
    A([Power on]) --> B["Initialise peripherals<br/>RGB LED, host UART, OLED, SD card,<br/>GPS + PPS, IMU (calibrate), LoRa"]
    B --> C["Load node ID from SD config,<br/>name and clear log files"]
    C --> D{"Boot errors?"}
    D -- yes --> E["Show errors on OLED,<br/>RGB red"] --> F
    D -- no --> F["Start two FreeRTOS tasks"]
    F --> S1
    F --> N1

    subgraph CORE0["Core 0: V2S task"]
        direction TB
        S1["Read GPS: time, position, course<br/>pair PPS edge with UTC second"] --> S2["Fuse GPS + IMU speed<br/>(Kalman filter, 100 Hz predict)"]
        S2 --> S3["Detect road events<br/>accident, bad terrain"]
        S3 --> S4["Read status, tasks, battery,<br/>destination, host message"]
        S4 --> S5["Write node state,<br/>raise message flags"]
        S5 --> S1
    end

    subgraph CORE1["Core 1: V2V task"]
        direction TB
        N1["Receive LoRa packet (if any)<br/>start back-off, drop repeats"] --> N1a{"Addressed to me<br/>or broadcast?"}
        N1a -- yes --> N1b["Read packet, update cluster<br/>(direct senders only), log RX"]
        N1a -- no --> N1c["Queue for forwarding<br/>(TTL − 1, not sent at 0)"]
        N1b --> N2
        N1c --> N2{"Forward queued and<br/>transmission due?"}
        N2 -- yes --> N3["Forward packet, log FW"] --> N4
        N2 -- no --> N4{"No forward queued and<br/>transmission due?"}
        N4 -- yes --> N5["Broadcast own packet +<br/>top-priority message,<br/>flash RGB, log TX"] --> N6
        N4 -- no --> N6["Remove peers silent<br/>for 30 s"]
        N6 --> N7["Send own data + peers<br/>to host (every 1 s)"]
        N7 --> N8["Update OLED and RGB,<br/>write SD logs"]
        N8 --> N1
    end

    TD["Transmission due: 15 s since last<br/>beacon or forward, back-off over,<br/>channel free. Forwards take<br/>priority over my beacon"]
    N4 ~~~ TD
    SH[("Shared: node state +<br/>message flags (mutex)")]
    CORE0 -. writes .-> SH
    CORE1 -. reads .-> SH
```
