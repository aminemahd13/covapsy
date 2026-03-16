# COVAPSY 2026 Rules — Researched Summary

**Official rules distilled into an implementation-oriented bullet document**  
**Event target:** 6th edition CoVAPSy, ENS Paris-Saclay  
**Prepared:** 2026-03-08  
**Audience:** autonomous 1/10-scale racing team / implementation planning

## Scope and source status

- I did not find a separate 2026 PDF rulebook. The current official source of truth is the COVAPSY rules page on the official event website, supported by the official homepage and the official GitHub repository.
- This document separates what is explicitly stated in the official rules from what is only a practical implication for your build.
- Where the published rules are slightly inconsistent, this document flags the inconsistency instead of silently smoothing it over.

## 1. High-impact takeaways

- The 2026 edition is scheduled for Saturday, April 11, 2026, at ENS Paris-Saclay. [S2]
- Only start and stop signals may be sent from the team to the vehicle during the race; telemetry from the vehicle back to the team is allowed. Sending behavioral commands to the vehicle is disqualifying. [S1]
- A 2026-specific relaxation exists: an off-car trackside computer is allowed for remote computation, but the team may not touch that computer once the start is announced. [S1]
- The car must be built on a Tamiya TT-02 chassis/motor kit, use a propulsion battery limited to 7.2 V NiMH and 5000 mAh maximum, and be able to drive both forward and reverse. [S1]
- The track layout is unknown until race day. Preloading the shape of the track into the car is forbidden; however, the car may learn the track through up to 3 setup laps. [S1]
- Homologation is not just a dimensional/safety check: the car must also demonstrate remote start/stop, straight-plus-turn driving without touching borders, reversing when blocked, and obstacle avoidance. [S1]
- The rear of the car must be LiDAR-visible. If the rear paint is not detectable, the referees may add off-white tape to the rear of the car. [S1]

## 2. Official rules in bullet points

### 2.1 Vehicle rules

- The vehicle must use a Tamiya TT-02 chassis/motor kit. [S1]
- The propulsion battery must be NiMH, 7.2 V, with a maximum capacity of 5000 mAh. [S1]
- A separate secondary battery is allowed for electronics, regardless of chemistry. [S1]
- Feeding the motor with a voltage higher than the propulsion battery voltage is not allowed. [S1]
- The body shell must cover more than 80% of the car; the jury estimates this informally rather than via a strict measurement process. [S1]
- Complete car dimensions, including all sensors and actuators, must stay within these limits: length strictly between 400 mm and 480 mm, width strictly between 160 mm and 190 mm, and height strictly between 115 mm and 190 mm. [S1, S4]
- The rear of the vehicle must present at least one solid rectangle 150 mm wide by 110 mm high; only a small gap under 10 mm in height is tolerated for ground clearance. [S1, S4]
- Transparent body shells are forbidden. A body whose main color is green, red, or gray is also forbidden. [S1]
- Rear visibility is checked with an RP-Lidar A2M12 or A2M8. If the rear is not detected, the organizers may add off-white tape to make it detectable. [S1]
- Removing the front-wheel drive is allowed if you want to trade traction for a tighter steering radius. [S1]
- The vehicle must be able to operate in forward and reverse. [S1]
- Minor steering-mechanics upgrades using commercial aluminum parts are allowed, but major chassis modifications beyond a few drilled holes require a prior request and acceptance by the other participating institutions; the request must include references or plans. [S1]
- A "typical car" kit is available as a resource, but it is a support option rather than a mandatory architecture. [S1, S4]

### 2.2 Team-to-car communication and off-board computing

- During competition, team-to-car communication must be limited to a start signal and a stop signal. [S1]
- Vehicle-to-team telemetry is allowed. [S1]
- Any command that modifies vehicle behavior during the run causes disqualification. [S1]
- For the 2026 edition, off-board computation on a trackside computer is explicitly allowed. [S1]
- Once the start is announced, the team may not touch the trackside computer. [S1]
- Practical implication: remote visualization or map optimization is compatible with the published rules only if the car remains autonomous after the start and no human-in-the-loop driving/planning commands are sent. [Inference from S1]

### 2.3 Track rules

- The shape of the track is unknown before race day. [S1]
- Providing information about the track shape to the vehicle is forbidden. Referees may verify compliance, for example on an auxiliary track. [S1]
- The car may learn the track by performing up to 3 setup laps. [S1]
- The track uses 200 mm-high borders. [S1]
- In one travel direction, the right border is green and the left border is red; in the opposite direction the color roles reverse. [S1]
- Borders are made from straight segments and circular arcs with curvature radius R >= 400 mm. [S1]
- The floor is a gray linoleum/PVC surface. [S1]
- The track width is greater than 800 mm at every point, but the track may contain obstacles inside the drivable space. [S1]
- The published color references are approximate: green RAL 6037, red RAL 3020, and a gray Gerflor flooring reference for the ground. The rules explicitly warn that borders may be marked by impacts, so perception must tolerate wear and variation. [S1]

### 2.4 Homologation requirements

- The published rules say homologation occurs in "4 steps", but the page actually lists 5 checks. For planning, treat all 5 listed checks as required. [S1]
- Check 1: validation of vehicle elements such as dimensions, battery, chassis, and LiDAR-detectable rear color/finish. [S1]
- Check 2: validation of remote start and remote stop. [S1]
- Check 3: demonstration that the vehicle can drive along a straight section plus a turn without touching the borders. [S1]
- Check 4: demonstration that the vehicle can reverse and restart when blocked against an obstacle, provided there is no vehicle behind it. [S1]
- Check 5: demonstration that the vehicle can avoid an obstacle approximately the size of a car on a section of track. [S1]
- The referees may still grant homologation with penalties for light rule violations; the page gives remote stop deficiencies and lack of reverse operation as examples. [S1]

### 2.5 Setup laps and SLAM-specific allowance

- For both races and qualifications, a car that uses SLAM may perform up to 3 setup laps without obstacles. [S1]
- Only after the car is placed on the starting grid are the obstacles or opponent vehicles added. [S1]
- Practical implication: mapping/localization during setup laps is explicitly contemplated by the official rules. [Inference from S1]

### 2.6 Qualification phase 1 — time trial

- Qualification phase 1 consists of 2 solo runs on track A with fixed obstacles roughly the size of a car. [S1]
- Each run is 2 laps. [S1]
- One run is clockwise and the other is counterclockwise. [S1]
- The best of the 2 times is retained. [S1]
- If the car fails to finish 2 laps in both runs, a time of 120 s is assigned if the car completed 1 lap, and 180 s if it did not even finish 1 lap. [S1]
- After this phase, the first car gets 25 points, the second 24, and so on down to a minimum of 5 points for any car with a time under 120 s. A car that completed only 1 lap and is assigned 120 s gets 1 point. [S1]

### 2.7 Qualification phase 2 — heat races

- Qualification phase 2 consists of 2 heat races with groups of 4 to 8 cars. [S1]
- The cars run 3 laps on track B and 3 laps on track C. [S1]
- One of those races is clockwise and the other is counterclockwise. [S1]
- The N pools are built from qualification ranking using the published modulo-style rule described on the official page. [S1]
- Each race awards 10 points to 1st place, 8 to 2nd, 6 to 3rd, 4 to cars that completed all 3 laps, and 1 to cars that did not finish. [S1]

### 2.8 Common procedure during race heats and finals

- Teams have 3 minutes to install their vehicle on the track. [S1]
- Vehicles are placed on the starting grid according to qualification results. [S1]
- After all teams declare they are ready, touching the vehicles is forbidden. [S1]
- The start signal is given orally by the referee. [S1]
- Arrival order is recorded after a predefined number of laps: by default 3 laps in qualification races and 5 laps in final races. [S1]
- A car that does not complete the defined number of laps is not classified for that race. [S1]

### 2.9 Removals, penalties, and disqualification conditions

- A vehicle showing clearly aggressive behavior toward opponents is removed/disqualified from the race. [S1]
- A vehicle that intentionally prevents another car from overtaking is also removed/disqualified. [S1]
- A vehicle immobilized on track for more than 10 seconds, when it is not being blocked by another car, is removed from the track. [S1]
- A vehicle that travels more than 2 meters in the wrong direction is removed from the track. [S1]
- Sending team-to-car commands beyond start/stop is disqualifying at the rules level. [S1]
- The rules also allow homologation penalties for light infractions before racing starts. [S1]

### 2.10 Final phases

- After qualifications, each car has 2 scores: coursesQualif (sum of the 2 qualification-race scores) and CLM+courseQualif (that same total plus the time-trial points). [S1]
- The finals are organized as 2 rounds ("manches"), each made of 3 races. [S1]
- The top 2 cars from each pool go to race 1 of each final round; the 3rd and 4th from each pool go to race 2; the others go to race 3. [S1]
- Starting grid order is determined by qualification ranking, using coursesQualif first and CLM+courseQualif as the tie-breaker for ex aequo cases. [S1]
- Final-race points are awarded as follows: 25, 18, 15, 12, 10, 8, 6, 4, 2, 1 from 1st through 10th place; a car that does not finish the required laps gets 0 points. [S1]
- After ranking the cars from the first race, points continue to be assigned to the cars from the second race as described on the official page. [S1]
- The final overall classification uses only the points from the 2 final races. [S1]
- If there is a tie in the final classification, CLM+courseQualif is used first; if still tied, coursesQualif is used. [S1]

### 2.11 Prizes

- The official page states that prizes are awarded to 1st, 2nd, and 3rd place overall. [S1]
- There is also an innovation prize. [S1]
- There is also a prize for the best-ranked team among the licence-level entrants. [S1]

## 3. What the rules do not mandate

- The rules do not mandate Raspberry Pi 5, STM32, ROS 2, Linux, Webots, RealSense, RPLIDAR as your onboard stack, or any particular software architecture. Those are engineering choices, not regulatory requirements. [S1, S4, S5]
- The rules do not mandate SLAM specifically; they only allow up to 3 setup laps and explicitly mention SLAM-capable cars in that context. [S1]
- The rules do not mandate a camera. They also do not mandate a LiDAR as the primary navigation sensor, although rear detectability is checked with an IR sensor hardware. [S1]
- The typical car page and simulator page are official resources for getting started, but they are not the competition rules themselves. [S4, S5]

## 4. Build-critical implications for your implementation plan

- Your car must have a reliable remote start/stop path that works under homologation conditions. This is not optional. [S1]
- Reverse maneuver capability is a hard competition requirement, not a "nice to have". [S1]
- Obstacle avoidance must work both in homologation and during racing. A pure lane-following solution without obstacle handling is non-compliant. [S1]
- Unknown-track operation matters: you cannot rely on a preloaded official map. Any map-based racing mode must be learned on site within the allowed setup-lap framework. [S1]
- Human-in-the-loop trackside control is not allowed. If you use offboard compute in 2026, the car still needs to remain autonomous and safe after the start signal. [S1]
- Rear detectability is a real rule item, so body design and paint choice are part of compliance. [S1]
- Because aggressive blocking and more than 2 m of wrong-way travel cause removal, recovery logic must be conservative and direction-aware. [S1]
- Because a car sitting still for more than 10 s without being blocked is removed, deadlock detection and fallback behavior matter strategically, not just technically. [S1]

## 5. Open points / ambiguities to verify early

- The official page describes homologation as "4 steps" but lists 5 checks. Treat the list itself as authoritative unless the organizers clarify otherwise. [S1]
- The page gives dimensional bounds through an image rather than a text table. Keep your mechanical design clearly inside the limits rather than exactly at a boundary. [S1, S4]
- The wording about final-round point continuation across races is concise and could be read in more than one procedural way. It does not change the safe engineering conclusion that finishing and classification are critical. [S1]
- Because the official site notes that the rules evolve year to year, re-check the rules page shortly before final freeze and before travel. [S1]

## 6. Sources reviewed

- **[S1]** Official rules page — COVAPSY / ENS Paris-Saclay — https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/Reglement_CourseVoituresAutonomes/
- **[S2]** Official event homepage — confirms 2026 date — https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/
- **[S3]** Official GitHub repository — rules/resources host — https://github.com/ajuton-ens/CourseVoituresAutonomesSaclay
- **[S4]** Official "typical car" page — supportive resource, plus mechanical context — https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/voiture_type/
- **[S5]** Official simulator page — supportive resource for 2025–2026 development — https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/simulateur/





# COVAPSY rear detection with IR sensors and LiDAR blind spots explained

The RPLIDAR A2M12 mounted atop a Tamiya TT-02 chassis loses **approximately 160° of its rear arc** (between ~100° and ~260°, counter-clockwise from front), leaving only the front 200° usable. In **this car**, the rear blind zone is covered with **IR rangefinders** connected to the STM32 analog inputs, rather than with the SRF10 ultrasonic module used in the official "voiture type" reference. Official documentation explicitly supports rear **IR telemeters** as a valid option. No specific Sharp sensor model is named in the official material. Below is a cleaned-up explanation aligned with an **IR-based rear detection setup**.

---

## The 160° LiDAR dead zone behind the car

The official COVAPSY Python programming guide (document 16088) states the problem precisely:

> *"Le lidar capte tout autour de lui, mais les mesures vers l'arrière (entre 100 et 260° environ) sont perturbées par l'habitacle de la voiture."*

The C/STM32 programming guide (document 16131) confirms with a shorter phrasing: *"Les mesures du Lidar vers l'arrière de la voiture sont perturbées par l'habitacle de la voiture."* Using the COVAPSY angle convention—**0° is the front of the car**, angles increase counter-clockwise—the blocked arc spans from roughly **100° (left-rear)** through 180° (directly behind) to **260° (right-rear)**. That is about **44% of the full 360° sweep** rendered unreliable by the body shell and internal structures intercepting the laser beam path.

The reference driving code explicitly discards rear data. The `conduite_autonome()` thread copies only the front arc:

```python
for i in range(-100, 101):
    tableau_lidar_mm[i] = acqui_tableau_lidar_mm[i]
````

This means teams working from the official codebase operate with a **200° forward-facing LiDAR field of view** and remain effectively blind rearward from LiDAR alone. The LiDAR data is stored in a `Data_Lidar_mm[360]` (C) or `tableau_lidar_mm[360]` (Python) array where each index corresponds to one degree. Reception uses interrupt-driven UART at **256,000 baud** (A2M12) or 115,200 baud (older A2M8) on USART1 (PA9 TX, PA10 RX), with 5-byte measurement packets containing quality, angle, and distance.

---

## Rear sensors: IR rangefinders are used for rear coverage

For **this implementation**, rear coverage is provided by **IR rangefinders**, not by an ultrasonic sensor. This is consistent with the official documentation, which explicitly allows rear **IR telemeters and/or ultrasonic** sensors.

The 3D-printed mounting documentation says the car can include *"une fixation propre des capteurs à l'avant (caméra Raspberry) comme à l'arrière (télémètres IR **et/ou** ultrason)"*. That wording matters: it confirms that **IR is an officially documented rear-sensing option**, not a hack or unsupported modification.

Two analog input pins are allocated in the STM32 configuration for rear IR sensing:

* **TELEMETRE INFRAROUGE 1** → PA1 (Analog, maps to ADC1_IN2)
* **TELEMETRE INFRAROUGE 2** → PA3 (Analog, maps to ADC1_IN4)

These inputs are the natural fit for an IR-based rear detection system. In practice, one sensor can cover the **left-rear** sector and the other the **right-rear** sector, giving the car a simple way to determine whether the area behind is clear enough for a reverse maneuver.

**No specific IR sensor model** is named in the official PDFs, the official voiture type BOM, or the GitHub repository documentation. In a 1/10-scale robotics context, common candidates are analog Sharp-style sensors such as:

* **Sharp GP2Y0A21YK0F** — roughly 10 to 80 cm
* **Sharp GP2Y0A02YK0F** — roughly 20 to 150 cm

Both produce analog voltages that are well suited to STM32 ADC acquisition. But that identification remains an inference: the official documentation leaves the exact IR model choice open to teams.

The important point for your car is straightforward: the LiDAR does **not** reliably see the rear, so the **rear IR sensors** provide the missing information needed for safe reverse behavior and rear-vehicle awareness.

---

## Complete STM32 NUCLEO-G431KB pin mapping and board architecture

The full CoVAPSy_STM32 configuration uses a **NUCLEO-G431KB** (ARM Cortex-M4 at up to 170 MHz, 128 KB Flash, 32 KB RAM, about €10.89) programmed with STM32CubeIDE v1.13.1. The complete pin allocation relevant to an **IR-based rear sensing setup** is:

| Peripheral           | Pin(s) | Interface type                                  |
| -------------------- | ------ | ----------------------------------------------- |
| LiDAR TX             | PA9    | USART1 TX                                       |
| LiDAR RX             | PA10   | USART1 RX                                       |
| LiDAR motor PWM      | PB5    | Timer 3 Ch.2 (10 MHz clock, 40 µs period)       |
| Propulsion ESC       | PA8    | PWM Timer 1 Ch.1 (1 MHz clock, 20 ms period)    |
| Steering servo       | PA11   | PWM Timer 1 Ch.4 (shared timer with propulsion) |
| IMU BNO055 SCL       | PA15   | I2C SCL                                         |
| IMU BNO055 SDA       | PB7    | I2C SDA                                         |
| OLED display SCL     | PA15   | I2C SCL (shared bus)                            |
| OLED display SDA     | PB7    | I2C SDA (shared bus)                            |
| IR rangefinder 1     | PA1    | Analog input (ADC1_IN2)                         |
| IR rangefinder 2     | PA3    | Analog input (ADC1_IN4)                         |
| Buzzer               | PB6    | PWM Timer 4 Ch.1                                |
| Optical fork encoder | PA0    | Timer 2 Ch.1 capture mode                       |

The electronics sit on **three custom PCBs** designed in Eagle (schematics and board files are available in the GitHub repo's `Hardware/` folder):

* **Carte HAT**: mounts directly on the Raspberry Pi 4 in standard HAT form factor, positioned near the center of the car.
* **Carte Interface**: located at the **rear of the car**. It houses the DC/DC converter (Murata OKI-78SR, 5 V 1.5 A from the 7.2 V NiMH battery), connectors to the two actuators (ESC and steering servo), battery input, and the rear sensor wiring path. In an **IR-based configuration**, this is where the rear IR sensors are routed into the STM32-side electronics.
* **Carte Mezzanine**: sits on the upper perforated plate and carries the OLED display (model TF051, I2C), push buttons, and buzzer. It connects to the HAT board.

The official PDF describes the integration as: *"une carte électronique a été conçue spécifiquement afin de permettre l'intégration de la carte NUCLEO-G431KB dans la voiture dotée des cartes interface (convertisseur DC/DC et connectiques vers les différents capteurs et actionneurs et la batterie) et mezzanine (écran, boutons, buzzer)."* Note that the display is an **OLED**, not a TFT.

---

## Reverse maneuver code and how rear IR sensing fits into it

Competition rules mandate reverse capability. Homologation step 4 explicitly verifies: *"vérification de l'aptitude du véhicule à repartir en marche arrière en cas de blocage contre un obstacle et en l'absence de véhicule derrière."* A car immobilized for more than **10 seconds** without being blocked by another car is removed. A car driving more than **2 meters in the wrong direction** is also removed. Non-functioning reverse earns a penalty but does not automatically disqualify.

The official Python `recule()` function implements the RC ESC's **two-step reverse protocol**:

```python
def recule():
    set_vitesse_m_s(-vitesse_max_m_s_hard)   # Step 1: Full brake
    time.sleep(0.2)
    set_vitesse_m_s(0)                       # Step 2: Back to neutral
    time.sleep(0.2)
    set_vitesse_m_s(-1)                      # Step 3: Engage reverse
```

The C library `CoVAPSy_moteurs` handles negative speed values through PWM pulse width calculation: pulse widths between **1.0–1.4 ms** command reverse, versus **1.5 ms neutral** and **1.6–2.0 ms forward**, at a 50 Hz PWM frequency (20 ms period). Maximum hardware speed is **8 m/s**, though software usually limits it to around **2 m/s**.

The reference autonomous driving algorithm triggers reverse when the LiDAR detects obstacles within **150 mm** at key angles:

```python
if tableau_lidar_mm[0] > 0 and tableau_lidar_mm[0] < 150:
    set_direction_degre(0); recule(); time.sleep(0.5)
elif tableau_lidar_mm[-30] > 0 and tableau_lidar_mm[-30] < 150:
    set_direction_degre(-18); recule(); time.sleep(0.5)
elif tableau_lidar_mm[30] > 0 and tableau_lidar_mm[30] < 150:
    set_direction_degre(+18); recule(); time.sleep(0.5)
```

Steering range is **±18°** mechanical limit. Normal forward driving uses proportional control:

```python
angle = 0.02 * (distance_at_60° - distance_at_-60°)
```

For an **IR-equipped car**, the missing safety condition before calling `recule()` is to verify that the rear is clear using the two rear IR sensors. In other words:

* LiDAR decides that reversing is needed because the front is blocked.
* Rear IR sensors decide whether reversing is **safe** because the rear LiDAR arc is blind.

That makes the IR sensors operationally important for passing the homologation condition *"en l'absence de véhicule derrière"* instead of merely assuming the rear is clear.

A typical logic pattern is:

```python
rear_clear = (ir_left_rear > threshold_mm) and (ir_right_rear > threshold_mm)

if front_blocked and rear_clear:
    recule()
else:
    stop_or_retry_turning()
```

The exact threshold depends on the chosen IR model and its calibration curve.

---

## IR sensor behavior and practical limitations

Compared with ultrasonic sensing, IR sensing has different strengths and weaknesses, which matter for this car:

* **Pros**

  * Fast analog readout through the STM32 ADC
  * Simple wiring
  * Compact mounting at the rear
  * Good for short-range occupancy detection behind the car

* **Cons**

  * Measurement quality depends strongly on target material, angle, and reflectivity
  * Nonlinear voltage-to-distance behavior
  * Reduced robustness on dark, absorbent, or highly angled surfaces
  * Usually a shorter and less uniform detection range than ultrasonic

That means the rear IR sensors are best treated as **rear occupancy detectors** or **short-range reverse safety sensors**, not as precision long-range rear mapping sensors.

In practice, the STM32 firmware should:

1. Sample both IR analog channels at a fixed rate.
2. Filter them with a moving average or median filter.
3. Convert ADC values into approximate distance or into a simpler binary state such as **rear-left occupied / rear-right occupied**.
4. Expose that state to the Raspberry Pi or directly use it in local reverse-protection logic.

---

## STM32-to-Raspberry Pi serial communication remains undocumented

The SU-Bolides GitHub repos (`Course_2024` and `Course_2025`) both contain a `CoVAPSy_STM32/` directory described as code that *"links the car sensors to the Raspberry Pi."* The repos are **97% C code** with Makefiles, confirming a standard STM32CubeIDE project structure. However, the actual source files (`main.c`, `main.h`, `.ioc`) could not be retrieved through web search tools and require direct repository cloning.

What can still be inferred from the architecture:

* **USART1 is occupied by the LiDAR** on PA9/PA10 at 256,000 baud.
* Rear **IR sensors** are read locally by the STM32 through **PA1 and PA3 ADC inputs**.
* Therefore, if the Raspberry Pi needs rear-detection information, the STM32 must forward either:

  * raw ADC values,
  * filtered distances,
  * or a simple rear-clear / rear-blocked status
    over a separate communication channel.
* The NUCLEO-G431KB has a built-in **ST-Link Virtual COM Port** connected to USART2 (PA2/PA3), which is a likely communication path to the Raspberry Pi over USB.
* The ROS-side serial nodes live in separate repos under the **SorbonneUniversityBolideContributors** organization and are not directly exposed in the official pedagogical material.

So while the **rear IR sensing concept is documented**, the exact STM32↔RPi serial protocol used by specific teams is still **team-specific** and must be determined from firmware source or live serial traces.

---

## Conclusion

The rear-detection story for **your car** is simple: the TT-02 body creates a **160° LiDAR shadow** from about **100° to 260°**, so LiDAR alone cannot safely determine whether the space behind the vehicle is free. In your setup, that blind region is covered by **rear IR sensors** connected to **PA1** and **PA3** on the STM32. This is fully consistent with the official COVAPSY documentation, which explicitly allows rear **IR telemeters and/or ultrasonic** sensing.

The base competition code still uses only the front LiDAR arc and triggers reverse when obstacles appear within **150 mm** ahead. But the homologation rule requires reversing only when there is **no vehicle behind**, so an IR-equipped car should use its rear IR sensors as the actual rear-clear check before engaging reverse. That gives you a coherent architecture:

* **front LiDAR** for forward navigation and front obstacle detection,
* **rear IR sensors** for blind-spot coverage and reverse safety,
* **STM32** for low-level sensor acquisition and actuator control,
* and optional **Raspberry Pi logic** for higher-level decision-making.

That is the correct way to reinterpret the standard COVAPSY blind-spot problem for a car that uses **IR rear sensing instead of ultrasonic**.

