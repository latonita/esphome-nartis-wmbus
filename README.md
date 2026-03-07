# Компонент экспериментальный и на реальных устройствах не тестировался 🙈 😂 
Ищу добровольцев для опытов

[СПОДЭС/DLMS/COSEM](https://github.com/latonita/esphome-dlms-cosem) •
[МЭК-61107/IEC-61107](https://github.com/latonita/esphome-iec61107-meter) •
[Энергомера МЭК/IEC](https://github.com/latonita/esphome-energomera-iec) •
[Энергомера CE](https://github.com/latonita/esphome-energomera-ce) •
[СПб ЗИП ЦЭ2727А](https://github.com/latonita/esphome-ce2727a-meter) •
[Ленэлектро ЛЕ-2](https://github.com/latonita/esphome-le2-meter) •
[Пульсар-М](https://github.com/latonita/esphome-pulsar-m) •
[Энергомера BLE](https://github.com/latonita/esphome-energomera-ble) •
[Nordic UART (BLE NUS)](https://github.com/latonita/esphome-nordic-uart-ble) •
**Нартис W-MBus (RF 433)**

# esphome-nartis-wmbus

Подключение ESPHome к счётчикам электроэнергии Нартис И300, И100 по радиоканалу RF 433 МГц. ESP32 + радиомодуль CMT2300A заменяет штатный CIU (Communication Interface Unit) и общается напрямую со счётчиком по протоколу DLMS/COSEM поверх Wireless M-Bus.

Три режима работы:
- **session** — активный опрос счётчика (AARQ → GET-запросы → RLRQ)
- **listen** — пассивное ожидание push-пакетов от счётчика
- **sniffer** — дамп сырых RF-пакетов в лог (для отладки)

# Оглавление
- [Функции](#функции)
  - [Реализованы](#реализованы)
  - [Возможные задачи на будущее](#возможные-задачи-на-будущее)
- [Аппаратное обеспечение](#аппаратное-обеспечение)
  - [Подключение CMT2300A к ESP32](#подключение-cmt2300a-к-esp32)
- [Установка](#установка)
- [Быстрый старт](#быстрый-старт)
- [Конфигурация хаба (nartis_wmbus)](#конфигурация-хаба-nartis_wmbus)
- [Сенсоры](#сенсоры)
  - [Числовой сенсор (sensor)](#числовой-сенсор-sensor)
  - [Текстовый сенсор (text_sensor)](#текстовый-сенсор-text_sensor)
- [Примеры конфигураций](#примеры-конфигураций)
  - [Трёхфазный счётчик — базовые показания](#трёхфазный-счётчик--базовые-показания)
  - [Режим сниффера](#режим-сниффера)
- [Диагностика и советы](#диагностика-и-советы)
- [Лицензия](#лицензия)

# Функции

## Реализованы
- Прямое подключение к счётчику Нартис по радиоканалу 433 МГц (без штатного CIU)
- Протокол DLMS/COSEM с аутентификацией LLS (пароль)
- Шифрование AES-128-GCM (DLMS Security Suite 0)
- Три режима работы: session, listen, sniffer
- Запрос произвольных OBIS-кодов с указанием DLMS-класса и атрибута
- Числовые и текстовые сенсоры
- Настраиваемый канал RF (4 канала)
- Настраиваемый ключ шифрования и system title
- Работа только на ESP32 (используется mbedtls для AES-GCM)

## Возможные задачи на будущее
- Парсинг push-пакетов в режиме listen (публикация сенсоров)
- Синхронизация времени
- Поддержка HLS-аутентификации
- Ввод STS-токенов предоплаты

Если готовы помочь тестированием — пишите на anton.viktorov@live.com.

---

# Аппаратное обеспечение

Требуется:
- **ESP32** (любая плата — DevKit, WROOM, WROVER и т.д.)
- **CMT2300A** — радиомодуль sub-GHz (CMOSTEK/HopeRF). Модули на базе CMT2300A продаются как отдельные платы.

> **Важно**: CMT2300A использует нестандартный 3-проводной SPI с двунаправленной линией SDIO. Стандартный SPI-контроллер ESP32 **не подходит** — используется программный (bit-banged) SPI через обычные GPIO.

## Подключение CMT2300A к ESP32

| CMT2300A | ESP32   | Описание                      |
|----------|---------|-------------------------------|
| SDIO     | GPIO 13 | Данные SPI (двунаправленная)  |
| SCLK     | GPIO 14 | Тактовый сигнал SPI           |
| CSB      | GPIO 27 | Chip Select (основной банк)   |
| FCSB     | GPIO 26 | FIFO Chip Select              |
| GPIO1    | GPIO 35 | Прерывание (опционально)      |
| VDD      | 3.3V    | Питание                       |
| GND      | GND     | Общий провод                  |

Номера GPIO указаны для примера — можно использовать любые свободные выводы ESP32.

---

## Установка

Добавьте внешний компонент в конфигурацию ESPHome:

```yaml
external_components:
  - source: github://latonita/esphome-nartis-wmbus
    components: [nartis_wmbus]
    refresh: 1s
```

---

## Быстрый старт

Минимальная конфигурация хаба и одного сенсора:

```yaml
external_components:
  - source: github://latonita/esphome-nartis-wmbus
    components: [nartis_wmbus]
    refresh: 1s

nartis_wmbus:
  pin_sdio: GPIO13
  pin_sclk: GPIO14
  pin_csb: GPIO27
  pin_fcsb: GPIO26
  update_interval: 60s

sensor:
  - platform: nartis_wmbus
    name: "Активная мощность"
    obis_code: "1.0.1.7.0.255"
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement
```

---

## Конфигурация хаба (`nartis_wmbus`)

```yaml
nartis_wmbus:
  pin_sdio: GPIO13
  pin_sclk: GPIO14
  pin_csb: GPIO27
  pin_fcsb: GPIO26
  pin_gpio1: GPIO35          # опционально, прерывание CMT2300A
  channel: 1                  # RF-канал 0-3
  decryption_key: "5A435A66755436363669526467504E48"
  system_title: "1122334455667788"
  mode: session               # session / listen / sniffer
  update_interval: 60s
```

Параметры:
- **pin_sdio** (**Required**) — GPIO для линии данных SPI (SDIO).
- **pin_sclk** (**Required**) — GPIO для тактового сигнала SPI.
- **pin_csb** (**Required**) — GPIO для chip select основного банка регистров.
- **pin_fcsb** (**Required**) — GPIO для chip select FIFO.
- **pin_gpio1** (*Optional*) — GPIO для линии прерывания CMT2300A.
- **channel** (*Optional*) — RF-канал (0–3). По умолчанию: 1.
  - 0: 431.8 МГц
  - 1: 433.9 МГц
  - 2: 433.5 МГц
  - 3: 435.3 МГц
- **decryption_key** (*Optional*) — AES-128 ключ шифрования, 32 hex-символа (16 байт). По умолчанию: стандартный ключ из прошивки.
- **system_title** (*Optional*) — DLMS system title, 16 hex-символов (8 байт). По умолчанию: `1122334455667788`.
- **mode** (*Optional*) — режим работы. По умолчанию: `session`.
  - `session` — активный опрос: устанавливает DLMS-сессию, запрашивает каждый OBIS-код, закрывает сессию.
  - `listen` — пассивный приём: ожидает push-пакеты от счётчика, выводит в лог. Парсинг данных — в разработке.
  - `sniffer` — дамп сырых пакетов: принимает все RF-пакеты и выводит hex-дамп в лог. Сенсоры не публикуются.
- **update_interval** (*Optional*) — период опроса (только для режима session). По умолчанию: 60s.

---

## Сенсоры

### Числовой сенсор (sensor)

```yaml
sensor:
  - platform: nartis_wmbus
    name: "Напряжение фаза A"
    obis_code: "1.0.32.7.0.255"
    class_id: 3
    attribute: 2
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement
```

Параметры:
- **obis_code** (**Required**) — OBIS-код объекта, 6 чисел через точку (например, `1.0.32.7.0.255`).
- **class_id** (*Optional*) — DLMS-класс объекта (3 = Register, 8 = Clock и т.д.). По умолчанию: 3.
- **attribute** (*Optional*) — индекс атрибута. По умолчанию: 2 (value).
- Остальные параметры — стандартные для [ESPHome sensor](https://esphome.io/components/sensor/).

### Текстовый сенсор (text_sensor)

```yaml
text_sensor:
  - platform: nartis_wmbus
    name: "Серийный номер"
    obis_code: "0.0.96.1.0.255"
    class_id: 1
    attribute: 2
```

Параметры:
- **obis_code** (**Required**) — OBIS-код объекта.
- **class_id** (*Optional*) — DLMS-класс. По умолчанию: 1.
- **attribute** (*Optional*) — индекс атрибута. По умолчанию: 2.
- Остальные параметры — стандартные для [ESPHome text_sensor](https://esphome.io/components/text_sensor/).

---

## Примеры конфигураций

### Трёхфазный счётчик — базовые показания

```yaml
external_components:
  - source: github://latonita/esphome-nartis-wmbus
    components: [nartis_wmbus]
    refresh: 1s

nartis_wmbus:
  pin_sdio: GPIO13
  pin_sclk: GPIO14
  pin_csb: GPIO27
  pin_fcsb: GPIO26
  channel: 1
  update_interval: 60s

sensor:
  # Напряжения
  - platform: nartis_wmbus
    name: "Напряжение фаза A"
    obis_code: "1.0.32.7.0.255"
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement

  - platform: nartis_wmbus
    name: "Напряжение фаза B"
    obis_code: "1.0.52.7.0.255"
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement

  - platform: nartis_wmbus
    name: "Напряжение фаза C"
    obis_code: "1.0.72.7.0.255"
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement

  # Токи
  - platform: nartis_wmbus
    name: "Ток фаза A"
    obis_code: "1.0.31.7.0.255"
    unit_of_measurement: A
    accuracy_decimals: 2
    device_class: current
    state_class: measurement

  - platform: nartis_wmbus
    name: "Ток фаза B"
    obis_code: "1.0.51.7.0.255"
    unit_of_measurement: A
    accuracy_decimals: 2
    device_class: current
    state_class: measurement

  - platform: nartis_wmbus
    name: "Ток фаза C"
    obis_code: "1.0.71.7.0.255"
    unit_of_measurement: A
    accuracy_decimals: 2
    device_class: current
    state_class: measurement

  # Мощность
  - platform: nartis_wmbus
    name: "Активная мощность (сумма)"
    obis_code: "1.0.1.7.0.255"
    unit_of_measurement: W
    accuracy_decimals: 1
    device_class: power
    state_class: measurement

  # Энергия
  - platform: nartis_wmbus
    name: "Энергия импорт (T0)"
    obis_code: "1.0.1.8.0.255"
    unit_of_measurement: kWh
    accuracy_decimals: 2
    device_class: energy
    state_class: total_increasing

  # Частота
  - platform: nartis_wmbus
    name: "Частота сети"
    obis_code: "1.0.14.7.0.255"
    unit_of_measurement: Hz
    accuracy_decimals: 2
    device_class: frequency
    state_class: measurement

text_sensor:
  - platform: nartis_wmbus
    name: "Серийный номер"
    obis_code: "0.0.96.1.0.255"
    class_id: 1
    attribute: 2
```

### Режим сниффера

Для отладки и анализа RF-пакетов:

```yaml
nartis_wmbus:
  pin_sdio: GPIO13
  pin_sclk: GPIO14
  pin_csb: GPIO27
  pin_fcsb: GPIO26
  mode: sniffer
  channel: 1
```

В этом режиме сенсоры не нужны — все принятые пакеты выводятся в лог ESP.

---

## Диагностика и советы

1. **Нет связи со счётчиком** — проверьте правильность подключения CMT2300A (особенно линию SDIO). В логе должно быть сообщение об успешной инициализации радиомодуля.

2. **Счётчик не отвечает на AARQ** — попробуйте другой RF-канал (0–3). Убедитесь, что счётчик находится в зоне действия радиосигнала.

3. **Ошибка расшифровки** — убедитесь, что ключ шифрования (`decryption_key`) соответствует вашему счётчику. Стандартный ключ работает для большинства неперенастроенных устройств.

4. **Используйте режим sniffer** для проверки наличия RF-пакетов на выбранном канале перед настройкой режима session.

5. **Уровень логирования** — для подробной диагностики добавьте:
```yaml
logger:
  level: DEBUG
  logs:
    nartis_wmbus: DEBUG
```

---

## Лицензия
MIT
